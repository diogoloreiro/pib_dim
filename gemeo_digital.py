#!/usr/bin/env python3
"""
gemeo_digital.py  —  Gêmeo Digital em Tempo Real
Sistema: Dimmer MC-8A + Lâmpada incandescente + ACS712 + DS18B20

Modos:
  • Físico  — recebe DATA do Arduino via serial; gêmeo roda em paralelo
  • Virtual — planta sintética (parâmetros configuráveis); sem Arduino

Funcionalidades:
  • Modelo 1ª ordem + atraso puro: dT/dt = (K·P_ef(u) − (T−T_amb)) / τ
  • RLS online com fator de esquecimento λ — adapta K e τ em tempo real
  • Alerta visual quando |T_real − T_gêmeo| > limiar configurável
  • Modo "What-if": desacopla a entrada do gêmeo da entrada física

Dependências:
    pip install pyserial matplotlib numpy customtkinter
"""

import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog, messagebox
import threading
import serial
import serial.tools.list_ports
import csv
import time
import os
from collections import deque
from datetime import datetime


import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import faulthandler
faulthandler.enable()

# ─── Aparência ────────────────────────────────────────────────────────────
ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")

matplotlib.rcParams.update({
    "figure.facecolor": "#ffffff", "axes.facecolor":  "#fdfdfd",
    "axes.edgecolor":   "#bdbdbd", "axes.labelcolor": "#37474f",
    "xtick.color":      "#607d8b", "ytick.color":     "#607d8b",
    "text.color":       "#37474f", "grid.color":      "#e0e0e0",
    "grid.linestyle":   "--",      "grid.alpha":       0.8,
    "axes.spines.top":  False,     "axes.spines.right": False,
    "font.family":      "sans-serif",
})

# ─── Constantes ───────────────────────────────────────────────────────────
HALF_CYCLE_US = 8333    # 60 Hz
MAX_POINTS    = 800
SEND_MS       = 50       # taxa do Arduino (ms)
TS_SIM        = SEND_MS / 1000.0

BAUD_RATE  = 115200
C_REAL     = "#c62828"   # vermelho escuro — temperatura real
C_TWIN     = "#1565c0"   # azul escuro     — gêmeo
C_K        = "#e65100"   # laranja escuro  — ganho K
C_TAU      = "#6a1b9a"   # roxo escuro     — constante τ
C_CURR     = "#00695c"   # teal escuro     — corrente
C_PWR      = "#4e342e"   # marrom          — potência
C_ERR      = "#ad1457"   # rosa escuro     — erro

# ─── Curva do dimmer ─────────────────────────────────────────────────────
_N      = 20001
_ALPHAS = np.linspace(0.0, np.pi, _N)
_POWERS = (np.pi - _ALPHAS + np.sin(2.0 * _ALPHAS) / 2.0) / np.pi


def _p_ef(u_pct: float) -> float:
    """Potência normalizada com não-linearidade residual do filamento."""
    p  = np.clip(u_pct / 100.0, 0.0, 1.0)
    return float(np.clip(p + 0.05 * np.sin(np.pi * p), 0.0, 1.0))


def pct_to_delay(pct: float) -> int:
    p = float(np.clip(pct / 100.0, 0.0, 1.0))
    a = float(np.interp(p, _POWERS[::-1], _ALPHAS[::-1]))
    return int(np.clip(a / np.pi * HALF_CYCLE_US, 500, 9600))


# ═══════════════════════════════════════════════════════════════════════════
# Modelo do gêmeo (1ª ordem + atraso puro, integração Euler)
# ═══════════════════════════════════════════════════════════════════════════
class TwinModel:
    """
    T(k+1) = T(k) + Ts * (K·P_ef(u(k−nk)) − (T(k)−T_amb)) / τ
    I(k)   = I_max · √P_ef(u(k))
    """

    def __init__(self, K: float = 44.0, tau: float = 220.0,
                 theta: float = 9.0, T_amb: float = 26.0,
                 I_max: float = 0.787, Ts: float = TS_SIM):
        self.K     = K
        self.tau   = tau
        self.theta = theta
        self.T_amb = T_amb
        self.I_max = I_max
        self.Ts    = Ts
        self.T     = T_amb
        n = max(1, round(theta / Ts))
        self._ubuf = deque([0.0] * n, maxlen=n)

    def step(self, u_pct: float) -> tuple:
        """Avança 1 passo. Retorna (T_pred, I_pred, u_delayed)."""
        self._ubuf.append(u_pct)
        u_d  = self._ubuf[0]
        p    = _p_ef(u_d)
        T_ss = self.T_amb + self.K * p
        self.T += (T_ss - self.T) / self.tau * self.Ts
        I = self.I_max * np.sqrt(p) if p > 0.0 else 0.0
        return self.T, I, u_d

    def sync(self, T_real: float):
        """Força o gêmeo a adotar a temperatura real (re-sincronização)."""
        self.T = T_real
        n = len(self._ubuf)
        self._ubuf = deque([0.0] * n, maxlen=n)

    def reset(self):
        self.T = self.T_amb
        n = max(1, round(self.theta / self.Ts))
        self._ubuf = deque([0.0] * n, maxlen=n)

    def set_params(self, K: float, tau: float):
        self.K   = max(0.1, K)
        self.tau = max(1.0, tau)


# ═══════════════════════════════════════════════════════════════════════════
# Estimador RLS (Recursive Least Squares) com fator de esquecimento
# Modelo: T(k) = a·T(k-1) + b·u_d(k-1)  →  a = exp(−Ts/τ),  b = K(1−a)
# ═══════════════════════════════════════════════════════════════════════════
class RLSEstimator:
    """
    RLS para ARX(1,1):  dT(k) = a·dT(k-1) + b·u_d(k-1)
      a = exp(−Ts_eff/τ),   b = K·(1−a),   dT = T − T_amb

    Para sistemas lentos (τ >> Ts_hw), decimamos as amostras antes
    de rodar o RLS — evita polo em ~0.9998 que torna o problema mal-condicionado.
    Ts_eff = Ts_hw × decimation  (padrão: 50 ms × 600 = 30 s).

    Nota: o regressor usa u_delayed/100 (linear). O TwinModel internamente
    aplica _p_ef(u) (+5 % sin), então K_est converge para um valor efetivo
    médio, não exatamente o K configurado no painel — diferença < 5 %.
    """

    def __init__(self, Ts: float, forgetting: float = 0.97,
                 decimation: int = 600,
                 K0: float = 44.0, tau0: float = 220.0):
        self.Ts_eff  = Ts * decimation        # período efetivo do RLS (30s)
        self.dec     = decimation
        self.lam     = forgetting
        self._cnt    = 0

        # Inicializa com parâmetros do ensaio de degrau (se disponível)
        a0 = np.exp(-self.Ts_eff / max(tau0, 1.0))
        b0 = K0 * (1.0 - a0)
        self.theta = np.array([a0, b0])
        self.P     = np.eye(2) * 200.0
        self._y_prev  = None
        self._ud_prev = None

    def update(self, y_real: float, u_delayed: float) -> tuple:
        """
        Atualiza a cada `decimation` chamadas.
        Retorna (K_est, tau_est, inovação).
        """
        self._cnt += 1
        if self._cnt % self.dec != 0:
            return self.K_est, self.tau_est, 0.0

        if self._y_prev is None:
            self._y_prev  = y_real
            self._ud_prev = u_delayed
            return self.K_est, self.tau_est, 0.0

        # Pula atualização quando a excitação é insuficiente.
        # Com u≈0 e phi=[y_prev, 0], o RLS empurra a→1 (τ→∞).
        excitation = abs(u_delayed) + abs(self._ud_prev)
        if excitation < 0.05:           # menos de 5 % de entrada
            self._y_prev  = y_real
            self._ud_prev = u_delayed
            return self.K_est, self.tau_est, 0.0

        phi   = np.array([self._y_prev, self._ud_prev])
        innov = y_real - phi @ self.theta
        denom = self.lam + phi @ self.P @ phi
        Kg    = (self.P @ phi) / denom
        self.theta = self.theta + Kg * innov
        self.P     = (np.eye(2) - np.outer(Kg, phi)) @ self.P / self.lam

        # Limites físicos: a ∈ (0.5, 0.9999),  b > 0
        self.theta[0] = np.clip(self.theta[0], 0.50, 0.9999)
        self.theta[1] = np.clip(self.theta[1], 1e-4, 100.0)

        # Regularização: impede P de explodir quando regressores são
        # quase colineares (u constante por longos períodos)
        trace = np.trace(self.P)
        if trace > 5e4:
            self.P *= (5e4 / trace)

        self._y_prev  = y_real
        self._ud_prev = u_delayed
        return self.K_est, self.tau_est, float(innov)

    @property
    def K_est(self) -> float:
        a, b = self.theta
        return float(b / max(1.0 - a, 1e-4))

    @property
    def tau_est(self) -> float:
        a = float(np.clip(self.theta[0], 1e-6, 0.9999))
        return float(-self.Ts_eff / np.log(a))

    def reset(self, K0: float = 44.0, tau0: float = 220.0):
        a0 = np.exp(-self.Ts_eff / max(tau0, 1.0))
        b0 = K0 * (1.0 - a0)
        self.theta   = np.array([a0, b0])
        self.P       = np.eye(2) * 200.0
        self._y_prev = self._ud_prev = None
        self._cnt    = 0


# ═══════════════════════════════════════════════════════════════════════════
# Planta Virtual (modo sem Arduino)
# Parâmetros ligeiramente diferentes do gêmeo para simular discrepância real
# ═══════════════════════════════════════════════════════════════════════════
class VirtualPlant:
    """Sistema de referência para o modo virtual."""

    def __init__(self, K=44.0, tau=220.0, theta=9.0,
                 T_amb=26.0, I_max=0.787, Ts=TS_SIM,
                 sigma_T=0.13, sigma_I=0.012):
        self._mdl     = TwinModel(K, tau, theta, T_amb, I_max, Ts)
        self._sigma_T = sigma_T
        self._sigma_I = sigma_I
        self._rng     = np.random.default_rng(seed=99)

    def step(self, u_pct: float) -> tuple:
        T, I, u_d = self._mdl.step(u_pct)
        T_noisy = T + self._rng.normal(0, self._sigma_T)
        I_noisy = max(0.0, I + self._rng.normal(0, self._sigma_I))
        return T_noisy, I_noisy, u_d

    def reset(self):
        self._mdl.reset()


# ═══════════════════════════════════════════════════════════════════════════
# Aplicação
# ═══════════════════════════════════════════════════════════════════════════
def _card(parent, title="", **kw):
    c = ctk.CTkFrame(parent, corner_radius=4, fg_color="#ffffff",
                     border_width=1, border_color="#e0e0e0", **kw)
    if title:
        ctk.CTkLabel(c, text=title,
                     font=ctk.CTkFont(size=9, weight="bold"),
                     text_color="#546e7a").pack(anchor="w", padx=10, pady=(7, 2))
    return c


class GemeoApp:

    def __init__(self, root: ctk.CTk):
        self.root = root
        self.root.title("Gêmeo Digital — Identificação de Sistema em Tempo Real")
        self.root.geometry("1340x760")
        self.root.minsize(1100, 640)

        # ── Modelos ──────────────────────────────────────────────────────
        self.twin  = TwinModel()
        # Ts_eff = 30s → ΔT/amostra ≈ 3°C >> ruído 0.13°C (SNR≈23)
        self.rls   = RLSEstimator(Ts=TS_SIM, decimation=600)
        self.plant = VirtualPlant()  # usado apenas no modo Virtual

        # ── Estado ───────────────────────────────────────────────────────
        self.ser:            serial.Serial | None = None
        self.serial_thread:  threading.Thread | None = None
        self.virtual_thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

        self.source      = ctk.StringVar(value="Virtual")
        self.rls_active  = ctk.BooleanVar(value=True)
        self.whatif_mode = ctk.BooleanVar(value=False)
        self.power_pct   = ctk.DoubleVar(value=0.0)
        self.whatif_pct  = ctk.DoubleVar(value=50.0)

        self.div_threshold = 3.0      # °C para alerta de divergência

        # ── Buffers de dados ─────────────────────────────────────────────
        self.t_buf    : deque = deque(maxlen=MAX_POINTS)
        self.T_real   : deque = deque(maxlen=MAX_POINTS)
        self.T_twin   : deque = deque(maxlen=MAX_POINTS)
        self.I_real   : deque = deque(maxlen=MAX_POINTS)
        self.I_twin   : deque = deque(maxlen=MAX_POINTS)
        self.K_buf    : deque = deque(maxlen=MAX_POINTS)
        self.tau_buf  : deque = deque(maxlen=MAX_POINTS)
        self.pwr_buf  : deque = deque(maxlen=MAX_POINTS)
        self.err_buf  : deque = deque(maxlen=MAX_POINTS)
        self.ts_buf   : deque = deque(maxlen=MAX_POINTS)
        self.t0 = 0.0

        # ── CSV ──────────────────────────────────────────────────────────
        self.csv_file   = None
        self.csv_writer = None

        # Fontes
        self.F_TITLE = ctk.CTkFont(family="Helvetica",   size=18, weight="bold")
        self.F_BIG   = ctk.CTkFont(family="Courier New", size=18, weight="bold")
        self.F_MED   = ctk.CTkFont(family="Courier New", size=12)
        self.F_SMALL = ctk.CTkFont(family="Helvetica",   size=11)
        self.F_LOG   = ctk.CTkFont(family="Courier New", size=9)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._start_virtual()   # inicia em modo Virtual por padrão

    # ─── UI ──────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # ── Sidebar ──────────────────────────────────────────────────────
        sb = ctk.CTkScrollableFrame(self.root, width=278, corner_radius=0,
                                     fg_color="#f5f6f7",
                                     scrollbar_button_color="#bdbdbd")
        sb.grid(row=0, column=0, sticky="nsew")

        ctk.CTkLabel(sb, text="Gêmeo Digital",
                     font=self.F_TITLE, text_color="#1a237e"
                     ).pack(padx=14, pady=(16, 0), anchor="w")
        ctk.CTkLabel(sb, text="Modelo 1ª Ordem + Atraso  ·  RLS Online",
                     font=self.F_SMALL, text_color="#607d8b"
                     ).pack(padx=14, pady=(0, 12), anchor="w")

        # Fonte de dados
        c = _card(sb, "FONTE DE DADOS")
        c.pack(fill="x", padx=10, pady=(0, 6))

        for label, value in [("Virtual (sem Arduino)", "Virtual"),
                              ("Físico (Arduino)", "Físico")]:
            ctk.CTkRadioButton(
                c, text=label, variable=self.source, value=value,
                command=self._on_source_change,
                font=self.F_SMALL
            ).pack(anchor="w", padx=12, pady=3)

        # Conexão (só aparece no modo Físico)
        self.conn_card = _card(sb, "CONEXÃO SERIAL")
        self.conn_card.pack(fill="x", padx=10, pady=(0, 6))

        self.port_var   = ctk.StringVar()
        self.port_combo = ctk.CTkComboBox(self.conn_card, variable=self.port_var,
                                           width=190, state="readonly")
        self.port_combo.pack(padx=10, pady=(4, 2))
        ctk.CTkButton(self.conn_card, text="Atualizar portas", height=26,
                      fg_color="#78909c", hover_color="#607d8b",
                      font=self.F_SMALL,
                      command=self._refresh_ports).pack(fill="x", padx=10, pady=2)
        self.btn_conn = ctk.CTkButton(self.conn_card, text="Conectar",
                                       fg_color="#0d47a1", hover_color="#1565c0",
                                       command=self._toggle_connect)
        self.btn_conn.pack(fill="x", padx=10, pady=(2, 4))

        ctk.CTkLabel(self.conn_card, text="CONTROLE DO ENSAIO",
                     font=ctk.CTkFont(size=9, weight="bold"),
                     text_color="#546e7a").pack(anchor="w", padx=10, pady=(6, 1))

        self.btn_start_phy = ctk.CTkButton(
            self.conn_card, text="Iniciar ensaio",
            fg_color="#1b5e20", hover_color="#2e7d32",
            command=self._start_physical, state="disabled")
        self.btn_start_phy.pack(fill="x", padx=10, pady=(2, 2))

        self.btn_stop_phy = ctk.CTkButton(
            self.conn_card, text="Encerrar ensaio",
            fg_color="#b71c1c", hover_color="#c62828",
            command=self._stop_physical, state="disabled")
        self.btn_stop_phy.pack(fill="x", padx=10, pady=(2, 10))

        # Potência / entrada
        c = _card(sb, "ENTRADA  u(t)  (%)")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.lbl_pwr = ctk.CTkLabel(c, text="0.0 %",
                                     font=self.F_BIG, text_color="#37474f")
        self.lbl_pwr.pack(pady=(4, 0))
        self.slider_pwr = ctk.CTkSlider(
            c, from_=0, to=100, variable=self.power_pct,
            command=self._on_power_change,
            button_color="#0d47a1", button_hover_color="#1565c0",
            progress_color="#1565c0")
        self.slider_pwr.pack(fill="x", padx=12, pady=(2, 10))

        # Parâmetros do gêmeo
        c = _card(sb, "PARÂMETROS DO MODELO  (K, τ, θ)")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self._param_sliders = {}
        params = [
            ("K (°C)",   "K",     10.0, 80.0,  44.0),
            ("τ (s)",    "tau",   30.0, 600.0, 220.0),
            ("θ (s)",    "theta",  0.0,  60.0,   9.0),
            ("T_amb(°C)","T_amb", 15.0,  40.0,  26.0),
        ]
        for label, key, lo, hi, default in params:
            row = ctk.CTkFrame(c, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=2)
            ctk.CTkLabel(row, text=label, width=70, font=self.F_SMALL,
                         anchor="w").pack(side="left")
            var = ctk.DoubleVar(value=default)
            self._param_sliders[key] = var
            lbl = ctk.CTkLabel(row, text=f"{default:.1f}", width=46,
                               font=self.F_SMALL, text_color="#607d8b")
            lbl.pack(side="right")
            sl = ctk.CTkSlider(row, from_=lo, to=hi, variable=var, width=110,
                               button_color="#0d47a1", button_hover_color="#1565c0",
                               progress_color="#90caf9",
                               command=lambda v, k=key, l=lbl:
                                   (l.configure(text=f"{float(v):.1f}"),
                                    self._on_param_change()))
            sl.pack(side="left", padx=4)

        ctk.CTkButton(c, text="Carregar parâmetros identificados", height=28,
                      fg_color="#01579b", hover_color="#0277bd",
                      font=self.F_SMALL,
                      command=self._load_from_analysis).pack(
                          fill="x", padx=10, pady=(6, 2))
        ctk.CTkButton(c, text="Aplicar parâmetros", height=28,
                      fg_color="#1b5e20", hover_color="#2e7d32",
                      font=self.F_SMALL,
                      command=self._apply_params).pack(
                          fill="x", padx=10, pady=(0, 4))
        ctk.CTkButton(c, text="Sincronizar gêmeo com real", height=28,
                      fg_color="#546e7a", hover_color="#607d8b",
                      font=self.F_SMALL,
                      command=self._sync_twin).pack(
                          fill="x", padx=10, pady=(0, 10))

        # Planta virtual (parâmetros do sistema simulado)
        self.plant_card = _card(sb, "PLANTA VIRTUAL  (sistema de referência)")
        self.plant_card.pack(fill="x", padx=10, pady=(0, 6))

        self._plant_sliders = {}
        for label, key, lo, hi, default in [
            ("K (°C)",  "K",    10.0, 80.0,  44.0),
            ("τ (s)",   "tau",  30.0, 600.0, 220.0),
        ]:
            row = ctk.CTkFrame(self.plant_card, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=2)
            ctk.CTkLabel(row, text=label, width=70, font=self.F_SMALL,
                         anchor="w").pack(side="left")
            var = ctk.DoubleVar(value=default)
            self._plant_sliders[key] = var
            lbl = ctk.CTkLabel(row, text=f"{default:.1f}", width=46,
                               font=self.F_SMALL, text_color="#607d8b")
            lbl.pack(side="right")
            sl = ctk.CTkSlider(row, from_=lo, to=hi, variable=var, width=110,
                               button_color="#4a148c", button_hover_color="#6a1b9a",
                               progress_color="#ce93d8",
                               command=lambda v, l=lbl:
                                   l.configure(text=f"{float(v):.1f}"))
            sl.pack(side="left", padx=4)

        ctk.CTkButton(self.plant_card, text="Aplicar planta", height=28,
                      fg_color="#4a148c", hover_color="#6a1b9a",
                      font=self.F_SMALL,
                      command=self._apply_plant).pack(
                          fill="x", padx=10, pady=(4, 10))

        # RLS
        c = _card(sb, "ESTIMAÇÃO RECURSIVA  (RLS)")
        c.pack(fill="x", padx=10, pady=(0, 6))

        ctk.CTkSwitch(c, text="Adaptação ativa", variable=self.rls_active,
                      font=self.F_SMALL,
                      progress_color="#1b5e20").pack(anchor="w", padx=12, pady=(4, 2))

        row = ctk.CTkFrame(c, fg_color="transparent")
        row.pack(fill="x", padx=10, pady=2)
        ctk.CTkLabel(row, text="λ (esquecimento):", font=self.F_SMALL,
                     width=110).pack(side="left")
        self.lam_var = ctk.DoubleVar(value=0.97)
        self.lbl_lam = ctk.CTkLabel(row, text="0.970",
                                     font=self.F_SMALL, text_color="#546e7a", width=40)
        self.lbl_lam.pack(side="right")
        ctk.CTkSlider(row, from_=0.80, to=0.999, variable=self.lam_var, width=72,
                      button_color="#1b5e20", button_hover_color="#2e7d32",
                      progress_color="#a5d6a7",
                      command=lambda v: (
                          self.lbl_lam.configure(text=f"{float(v):.3f}"),
                          setattr(self.rls, 'lam', float(v)))
                      ).pack(side="left", padx=4)

        ctk.CTkButton(c, text="Reiniciar estimador", height=28,
                      fg_color="#b71c1c", hover_color="#c62828",
                      font=self.F_SMALL,
                      command=self._reset_rls).pack(
                          fill="x", padx=10, pady=(4, 10))

        # What-if
        c = _card(sb, "SIMULAÇÃO WHAT-IF")
        c.pack(fill="x", padx=10, pady=(0, 6))

        ctk.CTkSwitch(c, text="Desacoplar entrada do modelo",
                      variable=self.whatif_mode,
                      font=self.F_SMALL,
                      progress_color="#4a148c").pack(anchor="w", padx=12, pady=(4, 2))

        row = ctk.CTkFrame(c, fg_color="transparent")
        row.pack(fill="x", padx=10, pady=2)
        ctk.CTkLabel(row, text="Entrada u(t) simulada:", width=130,
                     font=self.F_SMALL).pack(side="left")
        self.lbl_whatif = ctk.CTkLabel(row, text="50.0 %",
                                        font=self.F_SMALL,
                                        text_color="#6a1b9a", width=46)
        self.lbl_whatif.pack(side="right")
        ctk.CTkSlider(c, from_=0, to=100, variable=self.whatif_pct,
                      button_color="#4a148c", button_hover_color="#6a1b9a",
                      progress_color="#ce93d8",
                      command=lambda v: self.lbl_whatif.configure(
                          text=f"{float(v):.1f} %")
                      ).pack(fill="x", padx=12, pady=(0, 10))

        # Controles finais
        c = _card(sb, "DADOS")
        c.pack(fill="x", padx=10, pady=(0, 6))

        ctk.CTkButton(c, text="Exportar série temporal (CSV)",
                      fg_color="#0d47a1", hover_color="#1565c0",
                      font=self.F_SMALL,
                      command=self._export_csv).pack(
                          fill="x", padx=10, pady=(4, 2))
        ctk.CTkButton(c, text="Reiniciar gêmeo digital",
                      fg_color="#546e7a", hover_color="#607d8b",
                      font=self.F_SMALL,
                      command=self._reset_twin).pack(
                          fill="x", padx=10, pady=(2, 10))

        # Indicadores
        c = _card(sb, "VARIÁVEIS DE ESTADO")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.lbl_T_real  = ctk.CTkLabel(c, text="T real  : --- °C",
                                          font=self.F_MED, text_color=C_REAL,
                                          anchor="w")
        self.lbl_T_real.pack(fill="x", padx=10, pady=(4, 1))
        self.lbl_T_twin  = ctk.CTkLabel(c, text="T gêmeo : --- °C",
                                          font=self.F_MED, text_color=C_TWIN,
                                          anchor="w")
        self.lbl_T_twin.pack(fill="x", padx=10, pady=1)
        self.lbl_err     = ctk.CTkLabel(c, text="Erro    : --- °C",
                                          font=self.F_MED, text_color="#546e7a",
                                          anchor="w")
        self.lbl_err.pack(fill="x", padx=10, pady=1)
        self.lbl_K_rls   = ctk.CTkLabel(c, text="K  RLS  : ---",
                                          font=self.F_MED, text_color=C_K,
                                          anchor="w")
        self.lbl_K_rls.pack(fill="x", padx=10, pady=1)
        self.lbl_tau_rls = ctk.CTkLabel(c, text="τ  RLS  : --- s",
                                          font=self.F_MED, text_color=C_TAU,
                                          anchor="w")
        self.lbl_tau_rls.pack(fill="x", padx=10, pady=(1, 6))

        self.lbl_status = ctk.CTkLabel(sb, text="● Virtual",
                                        font=self.F_SMALL, text_color="#2e7d32")
        self.lbl_status.pack(padx=14, pady=(4, 16), anchor="w")

        # ── Área principal ────────────────────────────────────────────────
        main = ctk.CTkFrame(self.root, corner_radius=0, fg_color="#eef0f3")
        main.grid(row=0, column=1, sticky="nsew")
        main.grid_rowconfigure(0, weight=3)
        main.grid_rowconfigure(1, weight=1)
        main.grid_columnconfigure(0, weight=1)

        plot_frame = ctk.CTkFrame(main, corner_radius=6, fg_color="#ffffff",
                                  border_width=1, border_color="#e0e0e0")
        plot_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=(10, 4))

        self._build_plots(plot_frame)

        log_frame = ctk.CTkFrame(main, corner_radius=6, fg_color="#ffffff",
                                 border_width=1, border_color="#e0e0e0")
        log_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(4, 10))
        log_frame.grid_rowconfigure(1, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(log_frame, text="REGISTRO DE EVENTOS",
                     font=ctk.CTkFont(size=14, weight="bold"),
                     text_color="#607d8b"
                     ).grid(row=0, column=0, sticky="w", padx=10, pady=(4, 1))
        self.log_box = ctk.CTkTextbox(
            log_frame, font=self.F_LOG,
            fg_color="#f8f9fa", text_color="#263238",
            state="disabled", wrap="none")
        self.log_box.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))

        self._refresh_ports()
        self._on_source_change()

    def _build_plots(self, parent):
        self.fig = Figure(figsize=(9, 5.5), dpi=96)
        gs = GridSpec(3, 2, figure=self.fig,
                      left=0.07, right=0.94, top=0.93, bottom=0.08,
                      hspace=0.55, wspace=0.45)

        # ── Temperatura ──
        ax_T = self.fig.add_subplot(gs[0, :])
        ax_T.set_ylabel("Temperatura (°C)", fontsize=8)
        ax_T.set_title("Temperatura: Real vs Gêmeo", fontsize=9, pad=3,
                        color="#37474f")
        ax_T.grid(True)
        self.ln_T_real,  = ax_T.plot([], [], color=C_REAL, lw=1.8,
                                      label="Real")
        self.ln_T_twin,  = ax_T.plot([], [], color=C_TWIN, lw=1.5,
                                      ls="--", label="Gêmeo")
        # fill_between é recriado a cada update em ax_T.collections
        ax_T.legend(loc="upper left", facecolor="#ffffff",
                    edgecolor="#e0e0e0", fontsize=8)
        self.ax_T = ax_T

        # ── Corrente ──
        ax_I = self.fig.add_subplot(gs[1, 0])
        ax_I.set_ylabel("I RMS (A)", fontsize=8, color=C_CURR)
        ax_I.set_title("Corrente", fontsize=9, pad=3, color="#37474f")
        ax_I.tick_params(axis="y", labelcolor=C_CURR, labelsize=7)
        ax_I.grid(True)
        self.ln_I_real, = ax_I.plot([], [], color=C_CURR, lw=1.5,
                                     label="Real")
        self.ln_I_twin, = ax_I.plot([], [], color="#4dd0e1", lw=1.2,
                                     ls="--", label="Gêmeo")
        ax_I.legend(facecolor="#ffffff", edgecolor="#e0e0e0", fontsize=7)
        self.ax_I = ax_I

        # ── Potência e Erro ──
        ax_P = self.fig.add_subplot(gs[1, 1])
        ax_P.set_ylabel("Potência (%)", fontsize=8, color=C_PWR)
        ax_P.set_title("Potência + |Erro|", fontsize=9, pad=3, color="#37474f")
        ax_P.tick_params(axis="y", labelcolor=C_PWR, labelsize=7)
        ax_P.set_ylim(0, 105)
        ax_P.grid(True)
        self.ln_P, = ax_P.step([], [], color=C_PWR, lw=1.2, where="post")
        ax_E2      = ax_P.twinx()
        ax_E2.set_ylabel("|Erro| °C", fontsize=7, color=C_ERR)
        ax_E2.tick_params(axis="y", labelcolor=C_ERR, labelsize=7)
        self.ln_abs_err, = ax_E2.plot([], [], color=C_ERR, lw=1.0, alpha=0.8)
        self.ax_P  = ax_P
        self.ax_E2 = ax_E2

        # ── Parâmetros RLS ──
        ax_K = self.fig.add_subplot(gs[2, :])
        ax_K.set_xlabel("Tempo (s)", fontsize=8)
        ax_K.set_ylabel("K  (°C)", fontsize=8, color=C_K)
        ax_K.set_title("Parâmetros RLS em tempo real", fontsize=9, pad=3,
                        color="#37474f")
        ax_K.tick_params(axis="y", labelcolor=C_K, labelsize=7)
        ax_K.grid(True)
        ax_tau = ax_K.twinx()
        ax_tau.set_ylabel("τ  (s)", fontsize=8, color=C_TAU)
        ax_tau.tick_params(axis="y", labelcolor=C_TAU, labelsize=7)
        self.ln_K,   = ax_K.plot([], [], color=C_K,   lw=1.5, label="K  (RLS)")
        self.ln_tau, = ax_tau.plot([], [], color=C_TAU, lw=1.5,
                                    ls="--", label="τ (RLS)")

        # linhas de referência (iniciam nos parâmetros do gêmeo)
        self.hline_K   = ax_K.axhline(
            self._param_sliders["K"].get(), color=C_K, ls=":", lw=0.8, alpha=0.5)
        self.hline_tau = ax_tau.axhline(
            self._param_sliders["tau"].get(), color=C_TAU, ls=":", lw=0.8, alpha=0.5)

        lines = [self.ln_K, self.ln_tau]
        labels = [l.get_label() for l in lines]
        ax_K.legend(lines, labels, facecolor="#ffffff",
                    edgecolor="#e0e0e0", fontsize=8, loc="upper right")
        self.ax_K   = ax_K
        self.ax_tau = ax_tau

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ─── Controles ────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.configure(values=ports if ports else [""])
        self.port_combo.set(ports[0] if ports else "")

    def _on_source_change(self):
        if self.source.get() == "Virtual":
            self.conn_card.pack_forget()
            self.plant_card.pack(fill="x", padx=10, pady=(0, 6),
                                  after=None)   # já está visível
            self._start_virtual()
            self.lbl_status.configure(text="● Virtual", text_color="#2e7d32")
        else:
            self.plant_card.pack_forget()
            self.conn_card.pack(fill="x", padx=10, pady=(0, 6))
            self._stop_virtual()
            self.lbl_status.configure(text="● Físico (desconectado)",
                                       text_color="#ffa726")

    def _on_power_change(self, _=None):
        pwr = self.power_pct.get()
        self.lbl_pwr.configure(text=f"{pwr:.1f} %")
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f"SET:{pwr:.2f}\n".encode())
            except serial.SerialException:
                pass

    def _on_param_change(self):
        pass  # parâmetros só aplicados ao clicar "Aplicar"

    def _load_from_analysis(self):
        """
        Tenta carregar K, τ, θ do arquivo *_arx_params.txt gerado pela
        analise_ensaio.py. O usuário pode apontar para qualquer txt de parâmetros.
        Formato esperado (linha K_estatico=XX ou K=XX, tau=XX):
          Parâmetros identificados:  K=24.20  τ=219.97  θ=8.90
        """
        path = filedialog.askopenfilename(
            title="Selecione o arquivo de parâmetros identificados",
            filetypes=[("Texto", "*.txt"), ("Todos", "*.*")])
        if not path:
            return
        try:
            txt = open(path).read()
            K, tau, theta = None, None, None
            for line in txt.splitlines():
                l = line.lower().replace(" ", "")
                if "k=" in l or "k_estatico" in l or "ganho" in l:
                    for token in l.split():
                        if "k=" in token:
                            K = float(token.split("=")[1].strip(",°C/degrau"))
                if "τ=" in l or "tau=" in l or "constantadetempo" in l or "τ" in line.lower():
                    for token in line.split():
                        if "τ" in token or "tau" in token.lower():
                            try:
                                tau = float(token.split("=")[1].strip("s,"))
                            except Exception:
                                pass
                if "θ=" in l or "theta=" in l or "atraso" in l:
                    for token in line.split():
                        if "θ" in token or "theta" in token.lower():
                            try:
                                theta = float(token.split("=")[1].strip("s,"))
                            except Exception:
                                pass

            # Fallback: busca valores numéricos nas linhas-chave
            import re
            nums = re.findall(r"[-+]?\d*\.?\d+", txt)
            if nums and K is None:
                # Tenta padrão "K = XX.XX °C"
                for line in txt.splitlines():
                    if "K" in line and "=" in line and "°" in line:
                        m = re.search(r"=\s*([\d.]+)", line)
                        if m: K = float(m.group(1))
                    if ("τ" in line or "tau" in line.lower()) and "=" in line:
                        m = re.search(r"=\s*([\d.]+)", line)
                        if m: tau = float(m.group(1))
                    if ("θ" in line or "theta" in line.lower()) and "=" in line:
                        m = re.search(r"=\s*([\d.]+)", line)
                        if m: theta = float(m.group(1))

            loaded = []
            if K is not None:
                self._param_sliders["K"].set(min(80.0, max(10.0, K)))
                loaded.append(f"K={K:.2f}")
            if tau is not None:
                self._param_sliders["tau"].set(min(600.0, max(30.0, tau)))
                loaded.append(f"τ={tau:.1f}s")
            if theta is not None:
                self._param_sliders["theta"].set(min(60.0, max(0.0, theta)))
                loaded.append(f"θ={theta:.1f}s")
            if loaded:
                self._log(f"Parâmetros carregados: {', '.join(loaded)}")
                self._apply_params()
            else:
                self._log("Não encontrei parâmetros no arquivo. Ajuste manualmente.")
        except Exception as ex:
            self._log(f"Erro ao carregar: {ex}")

    def _apply_params(self):
        K     = self._param_sliders["K"].get()
        tau   = self._param_sliders["tau"].get()
        theta = self._param_sliders["theta"].get()
        T_amb = self._param_sliders["T_amb"].get()
        self.twin.K     = K
        self.twin.tau   = tau
        self.twin.theta = theta
        self.twin.T_amb = T_amb
        n = max(1, round(theta / TS_SIM))
        self.twin._ubuf = deque([0.0] * n, maxlen=n)
        self.hline_K.set_ydata([K])
        self.hline_tau.set_ydata([tau])
        self._log(f"Parâmetros gêmeo: K={K:.1f} τ={tau:.0f}s θ={theta:.1f}s")

    def _apply_plant(self):
        K   = self._plant_sliders["K"].get()
        tau = self._plant_sliders["tau"].get()
        self.plant._mdl.K   = K
        self.plant._mdl.tau = tau
        self.plant.reset()
        self._log(f"Planta virtual atualizada: K={K:.1f} τ={tau:.0f}s")

    def _sync_twin(self):
        if self.T_real:
            self.twin.sync(self.T_real[-1])
            self._log(f"Gêmeo sincronizado: T={self.T_real[-1]:.2f}°C")

    def _reset_twin(self):
        self.twin.reset()
        for buf in (self.t_buf, self.T_real, self.T_twin, self.I_real,
                    self.I_twin, self.K_buf, self.tau_buf, self.pwr_buf,
                    self.err_buf, self.ts_buf):
            buf.clear()
        self.t0 = 0.0
        self._log("Gêmeo reiniciado.")

    def _reset_rls(self):
        self.rls.reset()
        self._log("RLS reiniciado.")

    # ─── Modo Virtual ─────────────────────────────────────────────────────
    def _start_virtual(self):
        self._stop_evt.clear()
        if self.virtual_thread and self.virtual_thread.is_alive():
            return
        self.plant.reset()
        self.twin.reset()
        self.t0 = 0.0
        self.virtual_thread = threading.Thread(
            target=self._virtual_loop, daemon=True)
        self.virtual_thread.start()

    def _stop_virtual(self):
        self._stop_evt.set()

    def _virtual_loop(self):
        """Gera dados da planta virtual a ~20 Hz (50 ms/amostra)."""
        t_sim = 0.0
        while not self._stop_evt.is_set():
            pwr = self.power_pct.get()
            T_r, I_r, u_d = self.plant.step(pwr)
            t_sim += TS_SIM
            self.root.after(0, self._process_sample,
                            t_sim, pwr, T_r, I_r, u_d)
            time.sleep(TS_SIM)

    # ─── Modo Físico ──────────────────────────────────────────────────────
    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Erro", "Selecione uma porta.")
            return
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self._stop_evt.clear()
            self.serial_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self.serial_thread.start()
            self.btn_conn.configure(text="Desconectar",
                                    fg_color="#b71c1c", hover_color="#c62828")
            self.btn_start_phy.configure(state="normal")
            self.btn_stop_phy.configure(state="disabled")
            self.lbl_status.configure(text=f"● {port}", text_color="#2e7d32")
            self._log(f"Conectado: {port}")
        except serial.SerialException as e:
            messagebox.showerror("Erro", str(e))

    def _disconnect(self):
        self._stop_evt.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.btn_conn.configure(text="Conectar",
                                fg_color="#1565c0", hover_color="#1976d2")
        self.btn_start_phy.configure(state="disabled")
        self.btn_stop_phy.configure(state="disabled")
        self.lbl_status.configure(text="● Físico (desconectado)",
                                   text_color="#ffa726")
        self._log("Desconectado.")

    def _start_physical(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"START\n")
                self.btn_start_phy.configure(state="disabled")
                self.btn_stop_phy.configure(state="normal")
                self.lbl_status.configure(text="● Ensaio ativo",
                                           text_color="#2e7d32")
                self._log("START enviado ao Arduino.")
            except serial.SerialException as e:
                self._log(f"Erro ao enviar START: {e}")

    def _stop_physical(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"STOP\n")
                self.btn_start_phy.configure(state="normal")
                self.btn_stop_phy.configure(state="disabled")
                port = self.port_var.get()
                self.lbl_status.configure(text=f"● {port}",
                                           text_color="#ffa726")
                self._log("STOP enviado ao Arduino.")
            except serial.SerialException as e:
                self._log(f"Erro ao enviar STOP: {e}")

    def _serial_reader(self):
        while not self._stop_evt.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("ascii", errors="ignore").strip()
                    if line.startswith("DATA:"):
                        self.root.after(0, self._parse_serial, line)
            except serial.SerialException:
                break
            time.sleep(0.005)

    def _parse_serial(self, line: str):
        parts = line[5:].split(",")
        if len(parts) not in (4, 6):
            return
        try:
            ms, delay_us, irms, temp = (float(x) for x in parts[:4])
        except ValueError:
            return
        if self.t0 == 0.0:
            self.t0 = ms / 1000.0
        t   = ms / 1000.0 - self.t0
        pwr = float(np.interp(delay_us, [500, 9600], [100, 0]))  # rough inversion
        self.power_pct.set(pwr)
        self._process_sample(t, pwr, temp, irms, u_d=None)

    # ─── Núcleo do gêmeo ──────────────────────────────────────────────────
    def _process_sample(self, t: float, pwr: float,
                        T_real: float, I_real: float,
                        u_d: float | None):
        # Entrada do gêmeo: normal ou what-if
        u_twin = self.whatif_pct.get() if self.whatif_mode.get() else pwr

        T_tw, I_tw, u_delayed = self.twin.step(u_twin)

        # RLS trabalha com ΔT = T − T_amb e u normalizado (0-1)
        # Evita que T_amb (≈26°C) contamine os coeficientes b.
        # No modo what-if a entrada do gêmeo é fictícia — suspende o RLS para
        # não estimar parâmetros com um u que não excita o sistema real.
        dT_real = T_real - self.twin.T_amb
        rls_ok  = self.rls_active.get() and not self.whatif_mode.get()
        K_est, tau_est, innov = (
            self.rls.update(dT_real, u_delayed / 100.0)
            if rls_ok
            else (self.rls.K_est, self.rls.tau_est, 0.0)
        )
        # Propaga adaptação para o gêmeo
        if self.rls_active.get():
            self.twin.set_params(K_est, tau_est)

        err = T_real - T_tw

        # Buffers
        self.t_buf.append(t)
        self.T_real.append(T_real)
        self.T_twin.append(T_tw)
        self.I_real.append(I_real)
        self.I_twin.append(I_tw)
        self.K_buf.append(K_est)
        self.tau_buf.append(tau_est)
        self.pwr_buf.append(pwr)
        self.err_buf.append(abs(err))
        self.ts_buf.append(datetime.now().strftime("%H:%M:%S.%f")[:-3])

        # CSV
        if self.csv_writer:
            self.csv_writer.writerow([
                datetime.now().strftime("%H:%M:%S.%f")[:-3],
                f"{t:.3f}", f"{pwr:.1f}",
                f"{T_real:.3f}", f"{T_tw:.3f}",
                f"{I_real:.4f}", f"{I_tw:.4f}",
                f"{K_est:.4f}", f"{tau_est:.2f}", f"{err:.4f}"
            ])
            self.csv_file.flush()

        # Atualiza UI a cada ~10 amostras (500 ms)
        if len(self.t_buf) % 10 == 0:
            self._update_ui(T_real, T_tw, err, K_est, tau_est)

    def _update_ui(self, T_r, T_tw, err, K_est, tau_est):
        self.lbl_T_real.configure(text=f"T real  : {T_r:.2f} °C")
        self.lbl_T_twin.configure(text=f"T gêmeo : {T_tw:.2f} °C")

        err_color = "#ef5350" if abs(err) > self.div_threshold else "#aaaaaa"
        self.lbl_err.configure(text=f"Erro    : {err:+.2f} °C",
                                text_color=err_color)
        self.lbl_K_rls.configure(text=f"K  RLS  : {K_est:.2f}")
        self.lbl_tau_rls.configure(text=f"τ  RLS  : {tau_est:.1f} s")

        self._update_plots()

    def _update_plots(self):
        if not self.t_buf:
            return
        t  = list(self.t_buf)
        tr = list(self.T_real)
        tt = list(self.T_twin)

        # Temperatura
        self.ln_T_real.set_data(t, tr)
        self.ln_T_twin.set_data(t, tt)
        # Atualiza fill_between de forma segura
        for coll in list(self.ax_T.collections):
            coll.remove()
        self.ax_T.fill_between(t, tr, tt, color=C_ERR, alpha=0.22)
        self.ax_T.relim(); self.ax_T.autoscale_view()

        # Corrente
        self.ln_I_real.set_data(t, list(self.I_real))
        self.ln_I_twin.set_data(t, list(self.I_twin))
        self.ax_I.relim(); self.ax_I.autoscale_view()

        # Potência + erro
        self.ln_P.set_data(t, list(self.pwr_buf))
        self.ln_abs_err.set_data(t, list(self.err_buf))
        self.ax_P.relim();  self.ax_P.autoscale_view()
        self.ax_E2.relim(); self.ax_E2.autoscale_view()

        # RLS
        self.ln_K.set_data(t, list(self.K_buf))
        self.ln_tau.set_data(t, list(self.tau_buf))
        self.ax_K.relim();   self.ax_K.autoscale_view()
        self.ax_tau.relim(); self.ax_tau.autoscale_view()

        self.canvas.draw_idle()

    # ─── CSV ──────────────────────────────────────────────────────────────
    def _export_csv(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=f"gemeo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "tempo_s", "potencia_pct",
                        "T_real", "T_gemeo", "I_real", "I_gemeo",
                        "K_rls", "tau_rls", "erro_T"])
            for row in zip(
                self.ts_buf,
                self.t_buf, self.pwr_buf,
                self.T_real, self.T_twin,
                self.I_real, self.I_twin,
                self.K_buf, self.tau_buf, self.err_buf
            ):
                w.writerow([f"{v:.4f}" if isinstance(v, float) else v
                             for v in row])
        self._log(f"Exportado: {path}")

    # ─── Log ──────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{ts}]  {msg}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _on_close(self):
        self._stop_evt.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


# ─── Entry point ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    root = ctk.CTk()
    GemeoApp(root)
    root.mainloop()
