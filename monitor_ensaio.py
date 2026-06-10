#!/usr/bin/env python3
"""
monitor_ensaio.py  –  Interface para ensaio sem controlador.
Arduino MEGA + ACS712 + DS18B20 + Dimmer (ZC + DIM)

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
import signal
import subprocess
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# ─── Temas ───────────────────────────────────────────────────────────────────
_DARK = {
    "appearance": "dark",
    "sidebar": "#161616", "card": "#1e1e1e", "main": "#141414",
    "plot_card": "#1e1e1e", "log_card": "#1e1e1e",
    "log_fg": "#0d0d0d",   "log_text": "#80cbc4",
    "title_color": "#4fc3f7", "subtitle_color": "#546e7a",
    "status_default": "#546e7a",
    "plt_fig": "#1e1e1e",  "plt_ax": "#242424",
    "plt_edge": "#555555", "plt_label": "#cccccc",
    "plt_tick": "#aaaaaa", "plt_grid": "#333333",
    "ax_title": "#cccccc",
    "temp_line": "#ef5350", "curr_line": "#4fc3f7", "pwr_line": "#ffd54f",
    "volt_line": "#42a5f5", "pwr_w_line": "#ce93d8",
    "pwr_slider_btn": "#ffd54f", "pwr_slider_hover": "#ffca28",
    "pwr_slider_prog": "#e65100",
    "pwr_label_color": "#ffd54f",
    "status_ok": "#66bb6a",
    "rcparams": {
        "figure.facecolor": "#1e1e1e", "axes.facecolor":   "#242424",
        "axes.edgecolor":   "#555555", "axes.labelcolor":  "#cccccc",
        "xtick.color":      "#aaaaaa", "ytick.color":      "#aaaaaa",
        "text.color":       "#cccccc", "grid.color":       "#333333",
        "grid.linestyle":   "--",      "grid.alpha":        0.5,
    },
}

_LIGHT = {
    "appearance": "light",
    "sidebar": "#f5f6f7", "card": "#ffffff", "main": "#eef0f3",
    "plot_card": "#ffffff", "log_card": "#ffffff",
    "log_fg": "#f8f9fa",   "log_text": "#263238",
    "title_color": "#1a237e", "subtitle_color": "#607d8b",
    "status_default": "#78909c",
    "plt_fig": "#ffffff",  "plt_ax": "#fdfdfd",
    "plt_edge": "#bdbdbd", "plt_label": "#37474f",
    "plt_tick": "#607d8b", "plt_grid": "#e0e0e0",
    "ax_title": "#37474f",
    "temp_line": "#c62828", "curr_line": "#00695c", "pwr_line": "#e65100",
    "volt_line": "#1565c0", "pwr_w_line": "#6a1b9a",
    "pwr_slider_btn": "#0d47a1", "pwr_slider_hover": "#1565c0",
    "pwr_slider_prog": "#90caf9",
    "pwr_label_color": "#0d47a1",
    "status_ok": "#2e7d32",
    "rcparams": {
        "figure.facecolor": "#ffffff", "axes.facecolor":   "#fdfdfd",
        "axes.edgecolor":   "#bdbdbd", "axes.labelcolor":  "#37474f",
        "xtick.color":      "#607d8b", "ytick.color":      "#607d8b",
        "text.color":       "#37474f", "grid.color":       "#e0e0e0",
        "grid.linestyle":   "--",      "grid.alpha":        0.8,
    },
}

# Começa no modo claro
_TH = _LIGHT
ctk.set_appearance_mode(_TH["appearance"])
ctk.set_default_color_theme("blue")
matplotlib.rcParams.update(_TH["rcparams"])

# ─── Constantes ──────────────────────────────────────────────────────────────
HALF_CYCLE_US  = 8333
MIN_DELAY_US   = 500
MAX_DELAY_US   = HALF_CYCLE_US - 400
MAX_POINTS     = 600
BAUD_RATE      = 115200

# ─── Conversão Potência (%) ↔ delay_us ───────────────────────────────────────
_N      = 20001
_ALPHAS = np.linspace(0.0, np.pi, _N)
_POWERS = (np.pi - _ALPHAS + np.sin(2.0 * _ALPHAS) / 2.0) / np.pi


def power_pct_to_delay_us(pct: float) -> int:
    p     = float(np.clip(pct / 100.0, 0.0, 1.0))
    alpha = float(np.interp(p, _POWERS[::-1], _ALPHAS[::-1]))
    delay = int(alpha / np.pi * HALF_CYCLE_US)
    return int(np.clip(delay, MIN_DELAY_US, MAX_DELAY_US))


def delay_us_to_power_pct(delay: float) -> float:
    alpha = float(np.clip(delay, 0, HALF_CYCLE_US)) / HALF_CYCLE_US * np.pi
    p = (np.pi - alpha + np.sin(2.0 * alpha) / 2.0) / np.pi
    return float(np.clip(p * 100.0, 0.0, 100.0))


class EnsaioApp:
    def __init__(self, root: ctk.CTk):
        self.root  = root
        self._dark = False   # começa no modo claro
        self._alive = True
        self.root.title("Ensaio de Potência — Sem Controlador")
        self.root.geometry("1260x760")
        self.root.minsize(1000, 620)

        self.ser: serial.Serial | None = None
        self.serial_thread: threading.Thread | None = None
        self.sweep_thread:  threading.Thread | None = None
        self._stop_event = threading.Event()

        self.test_active   = False
        self.sweep_running = False
        self.prbs_running  = False
        self.prbs_thread:  threading.Thread | None = None
        self.csv_file      = None
        self.csv_writer    = None
        self.log_file      = None
        self._last_data_log  = 0.0
        self._last_raw_log   = 0.0
        self._plot_counter   = 0
        self._label_counter  = 0
        self._reconnecting   = False
        self.t0            = 0.0

        self._lut_I   = None
        self._lut_pct = None
        self.linearizar_ativo = ctk.BooleanVar(value=False)

        self.times:    deque = deque(maxlen=MAX_POINTS)
        self.powers:   deque = deque(maxlen=MAX_POINTS)
        self.currents: deque = deque(maxlen=MAX_POINTS)
        self.temps:    deque = deque(maxlen=MAX_POINTS)
        self.voltages: deque = deque(maxlen=MAX_POINTS)   # urms_V
        self.powers_w: deque = deque(maxlen=MAX_POINTS)   # prms_W

        self.F_TITLE = ctk.CTkFont(family="Helvetica",   size=16, weight="bold")
        self.F_BIG   = ctk.CTkFont(family="Courier New", size=22, weight="bold")
        self.F_MED   = ctk.CTkFont(family="Courier New", size=13)
        self.F_SMALL = ctk.CTkFont(family="Helvetica",   size=11)
        self.F_LOG   = ctk.CTkFont(family="Courier New", size=11)

        # refs para theme switching
        self._all_cards:  list = []
        self._theme_refs: dict = {}   # named widget refs

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_ports()
        self._periodic_flush()

    # ─── Card helper ─────────────────────────────────────────────────────────
    def _card(self, parent, title: str = "", **kw) -> ctk.CTkFrame:
        th = _LIGHT if not self._dark else _DARK
        card = ctk.CTkFrame(parent, corner_radius=6,
                            fg_color=th["card"],
                            border_width=1, border_color="#e0e0e0",
                            **kw)
        self._all_cards.append(card)
        if title:
            ctk.CTkLabel(card, text=title,
                         font=ctk.CTkFont(size=9, weight="bold"),
                         text_color="#607d8b").pack(anchor="w", padx=12, pady=(8, 2))
        return card

    # ─── UI ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        th = _LIGHT

        # ── Sidebar ──────────────────────────────────────────────────────────
        sidebar = ctk.CTkScrollableFrame(
            self.root, width=278, corner_radius=0,
            fg_color=th["sidebar"], scrollbar_button_color="#bdbdbd"
        )
        sidebar.grid(row=0, column=0, sticky="nsew")
        self._theme_refs["sidebar"] = sidebar

        # Cabeçalho
        hdr = ctk.CTkFrame(sidebar, fg_color="transparent")
        hdr.pack(fill="x", padx=14, pady=(18, 0))
        self._lbl_title = ctk.CTkLabel(hdr, text="Ensaio Dimmer",
                                        font=self.F_TITLE,
                                        text_color=th["title_color"])
        self._lbl_title.pack(side="left", anchor="w")
        self._btn_theme = ctk.CTkButton(
            hdr, text="Escuro", width=58, height=24,
            fg_color="#78909c", hover_color="#607d8b",
            font=ctk.CTkFont(size=10),
            command=self._toggle_theme)
        self._btn_theme.pack(side="right", anchor="e")

        self._lbl_subtitle = ctk.CTkLabel(sidebar,
                                           text="Malha aberta  ·  ACS712 + DS18B20",
                                           font=self.F_SMALL,
                                           text_color=th["subtitle_color"])
        self._lbl_subtitle.pack(padx=14, pady=(2, 14), anchor="w")

        # ── Conexão ──
        c = self._card(sidebar, "CONEXÃO SERIAL")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.port_var   = ctk.StringVar()
        self.port_combo = ctk.CTkComboBox(c, variable=self.port_var,
                                          width=210, state="readonly")
        self.port_combo.pack(padx=12, pady=(4, 2))

        row = ctk.CTkFrame(c, fg_color="transparent")
        row.pack(fill="x", padx=12, pady=2)
        ctk.CTkLabel(row, text="Baud:", font=self.F_SMALL, width=44).pack(side="left")
        self.baud_var = ctk.StringVar(value=str(BAUD_RATE))
        ctk.CTkEntry(row, textvariable=self.baud_var, width=100).pack(side="left", padx=4)
        ctk.CTkButton(row, text="↻", width=36,
                      fg_color="#78909c", hover_color="#607d8b",
                      command=self._refresh_ports).pack(side="right")

        self.btn_connect = ctk.CTkButton(
            c, text="Conectar",
            fg_color="#0d47a1", hover_color="#1565c0",
            command=self._toggle_connect)
        self.btn_connect.pack(fill="x", padx=12, pady=(4, 12))

        # ── Potência ──
        c = self._card(sidebar, "ENTRADA  u(t)  (%)")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.power_var   = ctk.DoubleVar(value=50.0)
        self._lbl_pwr_big = ctk.CTkLabel(c, text="50.0 %",
                                          font=self.F_BIG,
                                          text_color=th["pwr_label_color"])
        self._lbl_pwr_big.pack(pady=(6, 0))

        self.slider_power = ctk.CTkSlider(
            c, from_=15, to=100, variable=self.power_var,
            command=self._on_power_change,
            button_color=th["pwr_slider_btn"],
            button_hover_color=th["pwr_slider_hover"],
            progress_color=th["pwr_slider_prog"])
        self.slider_power.pack(fill="x", padx=14, pady=(4, 2))

        row = ctk.CTkFrame(c, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(0, 10))
        ctk.CTkLabel(row, text="15 % = mínimo",
                     font=self.F_SMALL, text_color="#78909c").pack(side="left")
        ctk.CTkLabel(row, text="100 % = máx",
                     font=self.F_SMALL, text_color="#78909c").pack(side="right")

        # ── Linearização ──
        c = self._card(sidebar, "LINEARIZAÇÃO DO DIMMER")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.lbl_lut = ctk.CTkLabel(c, text="LUT: não carregada",
                                     font=self.F_SMALL, text_color="#78909c",
                                     anchor="w")
        self.lbl_lut.pack(fill="x", padx=12, pady=(4, 2))

        ctk.CTkButton(c, text="Carregar LUT (CSV)", height=28,
                      fg_color="#1b5e20", hover_color="#2e7d32",
                      font=self.F_SMALL,
                      command=self._carregar_lut).pack(fill="x", padx=12, pady=2)

        ctk.CTkSwitch(c, text="Ativar linearização",
                      variable=self.linearizar_ativo,
                      font=self.F_SMALL,
                      progress_color="#1b5e20").pack(anchor="w", padx=12, pady=(4, 10))

        # ── Leituras ──
        c = self._card(sidebar, "VARIÁVEIS DE ESTADO")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.lbl_temp   = ctk.CTkLabel(c, text="T       : --- °C",
                                        font=self.F_MED, text_color="#c62828",
                                        anchor="w")
        self.lbl_temp.pack(fill="x", padx=12, pady=(4, 2))
        self.lbl_irms   = ctk.CTkLabel(c, text="I RMS   : --- A",
                                        font=self.F_MED, text_color="#00695c",
                                        anchor="w")
        self.lbl_irms.pack(fill="x", padx=12, pady=2)
        self.lbl_urms   = ctk.CTkLabel(c, text="U RMS   : --- V",
                                        font=self.F_MED, text_color="#1565c0",
                                        anchor="w")
        self.lbl_urms.pack(fill="x", padx=12, pady=2)
        self.lbl_pwr_fb = ctk.CTkLabel(c, text="u(t)    : --- %",
                                        font=self.F_MED, text_color="#e65100",
                                        anchor="w")
        self.lbl_pwr_fb.pack(fill="x", padx=12, pady=2)
        self.lbl_prms   = ctk.CTkLabel(c, text="P real  : --- W",
                                        font=self.F_MED, text_color="#6a1b9a",
                                        anchor="w")
        self.lbl_prms.pack(fill="x", padx=12, pady=(2, 10))

        # ── Controles ──
        c = self._card(sidebar, "CONTROLE DO ENSAIO")
        c.pack(fill="x", padx=10, pady=(0, 6))

        self.btn_start = ctk.CTkButton(
            c, text="Iniciar ensaio",
            fg_color="#1b5e20", hover_color="#2e7d32",
            command=self._start_test, state="disabled")
        self.btn_start.pack(fill="x", padx=12, pady=(4, 2))

        self.btn_stop = ctk.CTkButton(
            c, text="Encerrar ensaio",
            fg_color="#b71c1c", hover_color="#c62828",
            command=self._stop_test, state="disabled")
        self.btn_stop.pack(fill="x", padx=12, pady=2)

        self.btn_cal = ctk.CTkButton(
            c, text="Calibrar ACS712 (sem carga)",
            fg_color="#546e7a", hover_color="#607d8b",
            command=self._calibrate, state="disabled")
        self.btn_cal.pack(fill="x", padx=12, pady=2)

        self.btn_save_csv = ctk.CTkButton(
            c, text="Exportar CSV",
            fg_color="#0d47a1", hover_color="#1565c0",
            command=self._export_csv, state="disabled")
        self.btn_save_csv.pack(fill="x", padx=12, pady=(2, 4))

        ctk.CTkButton(
            c, text="Exportar log",
            fg_color="#546e7a", hover_color="#607d8b",
            font=self.F_SMALL,
            command=self._export_log).pack(fill="x", padx=12, pady=(2, 4))

        ctk.CTkButton(
            c, text="Reset geral",
            fg_color="#4a148c", hover_color="#6a1b9a",
            font=self.F_SMALL,
            command=self._reset_geral).pack(fill="x", padx=12, pady=(2, 12))

        # ── Varredura ──
        c = self._card(sidebar, "VARREDURA AUTOMÁTICA  (%)")
        c.pack(fill="x", padx=10, pady=(0, 6))

        for label, attr, default in [
            ("De (%):",      "sweep_from",  "10"),
            ("Até (%):",     "sweep_to",   "100"),
            ("Passo (%):",   "sweep_step",   "5"),
            ("Espera (s):",  "sweep_dwell",  "30"),
        ]:
            row = ctk.CTkFrame(c, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=2)
            ctk.CTkLabel(row, text=label, width=82,
                         font=self.F_SMALL, anchor="w").pack(side="left")
            var = ctk.StringVar(value=default)
            setattr(self, f"{attr}_var", var)
            ctk.CTkEntry(row, textvariable=var, width=80).pack(side="right")

        self.btn_sweep = ctk.CTkButton(
            c, text="Iniciar varredura",
            fg_color="#4a148c", hover_color="#6a1b9a",
            command=self._toggle_sweep, state="disabled")
        self.btn_sweep.pack(fill="x", padx=12, pady=(6, 12))

        # ── PRBS ──
        c = self._card(sidebar, "PRBS  (identificação ARX)")
        c.pack(fill="x", padx=10, pady=(0, 6))

        for label, attr, default in [
            ("u_min (%):",      "prbs_umin",      "20"),
            ("u_max (%):",      "prbs_umax",      "70"),
            ("Intervalo (s):",  "prbs_interval",  "60"),
        ]:
            row = ctk.CTkFrame(c, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=2)
            ctk.CTkLabel(row, text=label, width=94,
                         font=self.F_SMALL, anchor="w").pack(side="left")
            var = ctk.StringVar(value=default)
            setattr(self, f"{attr}_var", var)
            ctk.CTkEntry(row, textvariable=var, width=70).pack(side="right")

        row = ctk.CTkFrame(c, fg_color="transparent")
        row.pack(fill="x", padx=12, pady=2)
        ctk.CTkLabel(row, text="Bits (período):",
                     font=self.F_SMALL, width=94, anchor="w").pack(side="left")
        self.prbs_bits_var = ctk.StringVar(value="7")
        ctk.CTkComboBox(row, variable=self.prbs_bits_var,
                        values=["5  (31 amostras)",
                                "6  (63 amostras)",
                                "7  (127 amostras)"],
                        width=130, state="readonly").pack(side="right")

        self.btn_prbs = ctk.CTkButton(
            c, text="Iniciar PRBS",
            fg_color="#006064", hover_color="#00838f",
            command=self._toggle_prbs, state="disabled")
        self.btn_prbs.pack(fill="x", padx=12, pady=(6, 12))

        # Status
        self.lbl_status = ctk.CTkLabel(
            sidebar, text="● Desconectado",
            font=self.F_SMALL, text_color=th["status_default"])
        self.lbl_status.pack(padx=14, pady=(4, 16), anchor="w")

        # ── Área principal ────────────────────────────────────────────────────
        self._main_frame = ctk.CTkFrame(self.root, corner_radius=0,
                                         fg_color=th["main"])
        self._main_frame.grid(row=0, column=1, sticky="nsew")
        self._main_frame.grid_rowconfigure(0, weight=3)
        self._main_frame.grid_rowconfigure(1, weight=1)
        self._main_frame.grid_columnconfigure(0, weight=1)

        self._plot_card = ctk.CTkFrame(self._main_frame, corner_radius=6,
                                        fg_color=th["plot_card"],
                                        border_width=1, border_color="#e0e0e0")
        self._plot_card.grid(row=0, column=0, sticky="nsew", padx=10, pady=(10, 4))

        self._build_plots(self._plot_card)

        self._log_card = ctk.CTkFrame(self._main_frame, corner_radius=6,
                                       fg_color=th["log_card"],
                                       border_width=1, border_color="#e0e0e0")
        self._log_card.grid(row=1, column=0, sticky="nsew", padx=10, pady=(4, 10))
        self._log_card.grid_rowconfigure(1, weight=1)
        self._log_card.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self._log_card, text="REGISTRO DE EVENTOS",
                     font=ctk.CTkFont(size=9, weight="bold"),
                     text_color="#607d8b"
                     ).grid(row=0, column=0, sticky="w", padx=10, pady=(4, 1))

        self.log_box = ctk.CTkTextbox(
            self._log_card, font=self.F_LOG,
            fg_color=th["log_fg"], text_color=th["log_text"],
            state="disabled", wrap="none")
        self.log_box.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))

    # ─── Gráficos ─────────────────────────────────────────────────────────────
    def _build_plots(self, parent):
        from matplotlib.gridspec import GridSpec
        th = _LIGHT if not self._dark else _DARK
        self.fig = Figure(figsize=(9.0, 6.5), dpi=96, facecolor=th["plt_fig"])
        gs = GridSpec(3, 2, figure=self.fig,
                      left=0.08, right=0.96, top=0.94, bottom=0.07,
                      hspace=0.62, wspace=0.42)

        def _ax(spec, title, ylabel, ycolor, xlabel=""):
            ax = self.fig.add_subplot(spec)
            ax.set_facecolor(th["plt_ax"])
            ax.set_title(title,  color=th["ax_title"],  fontsize=9, pad=3)
            ax.set_ylabel(ylabel, color=ycolor,           fontsize=8)
            if xlabel:
                ax.set_xlabel(xlabel, color=th["plt_label"], fontsize=8)
            ax.tick_params(axis="y", labelcolor=ycolor,       labelsize=7)
            ax.tick_params(axis="x", colors=th["plt_tick"],   labelsize=7)
            ax.grid(True, color=th["plt_grid"], linewidth=0.6)
            for sp in ax.spines.values():
                sp.set_edgecolor(th["plt_edge"])
            return ax

        # Linha 0 — Temperatura (largura total)
        self.ax_temp = _ax(gs[0, :], "Temperatura  T(t)",
                           "Temp (°C)", th["temp_line"])
        self.line_t, = self.ax_temp.plot([], [], color=th["temp_line"], lw=1.8)

        # Linha 1 — Corrente | Tensão
        self.ax_i = _ax(gs[1, 0], "Corrente  I(t)",
                        "I RMS (A)", th["curr_line"])
        self.line_i, = self.ax_i.plot([], [], color=th["curr_line"], lw=1.5)

        self.ax_v = _ax(gs[1, 1], "Tensão  U(t)",
                        "U RMS (V)", th["volt_line"])
        self.line_v, = self.ax_v.plot([], [], color=th["volt_line"], lw=1.5)

        # Linha 2 — Potência% | Potência W
        self.ax_p = _ax(gs[2, 0], "Sinal de controle  u(t)",
                        "u(t)  (%)", th["pwr_line"], xlabel="Tempo (s)")
        self.line_p, = self.ax_p.step([], [], color=th["pwr_line"], lw=1.5, where="post")

        self.ax_w = _ax(gs[2, 1], "Potência ativa  P(t)",
                        "P (W)", th["pwr_w_line"], xlabel="Tempo (s)")
        self.line_w, = self.ax_w.plot([], [], color=th["pwr_w_line"], lw=1.5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

    # ─── Tema ────────────────────────────────────────────────────────────────
    def _toggle_theme(self):
        self._dark = not self._dark
        th = _DARK if self._dark else _LIGHT
        ctk.set_appearance_mode(th["appearance"])
        matplotlib.rcParams.update(th["rcparams"])

        # Sidebar / main frames
        self._theme_refs["sidebar"].configure(fg_color=th["sidebar"])
        self._main_frame.configure(fg_color=th["main"])
        self._plot_card.configure(fg_color=th["plot_card"])
        self._log_card.configure(fg_color=th["log_card"])

        # Cards
        for card in self._all_cards:
            try:
                card.configure(fg_color=th["card"])
            except Exception:
                pass

        # Log box
        self.log_box.configure(fg_color=th["log_fg"], text_color=th["log_text"])

        # Título / subtítulo
        self._lbl_title.configure(text_color=th["title_color"])
        self._lbl_subtitle.configure(text_color=th["subtitle_color"])

        # Status
        self.lbl_status.configure(text_color=th["status_default"])

        # Botão de tema
        self._btn_theme.configure(text="Claro" if self._dark else "Escuro")

        # Slider de potência
        self.slider_power.configure(
            button_color=th["pwr_slider_btn"],
            button_hover_color=th["pwr_slider_hover"],
            progress_color=th["pwr_slider_prog"])
        self._lbl_pwr_big.configure(text_color=th["pwr_label_color"])

        # Plots — atualiza cores in-place
        self._apply_plot_theme(th)

    def _apply_plot_theme(self, th: dict):
        self.fig.patch.set_facecolor(th["plt_fig"])
        specs = [
            (self.ax_temp, th["temp_line"],  "line_t"),
            (self.ax_i,    th["curr_line"],  "line_i"),
            (self.ax_v,    th["volt_line"],  "line_v"),
            (self.ax_p,    th["pwr_line"],   "line_p"),
            (self.ax_w,    th["pwr_w_line"], "line_w"),
        ]
        for ax, line_color, attr in specs:
            ax.set_facecolor(th["plt_ax"])
            ax.title.set_color(th["ax_title"])
            ax.xaxis.label.set_color(th["plt_label"])
            ax.yaxis.label.set_color(line_color)
            ax.tick_params(axis="y", labelcolor=line_color)
            ax.tick_params(axis="x", colors=th["plt_tick"])
            ax.grid(True, color=th["plt_grid"], linewidth=0.6)
            for spine in ax.spines.values():
                spine.set_edgecolor(th["plt_edge"])
            getattr(self, attr).set_color(line_color)
        if self._alive:
            self.canvas.draw_idle()

    # ─── Portas ───────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        all_ports = serial.tools.list_ports.comports()
        ports = [p.device for p in all_ports
                 if not (p.device.startswith("/dev/ttyS") and
                         p.description in ("n/a", ""))]
        if not ports:
            ports = [p.device for p in all_ports]
        self.port_combo.configure(values=ports if ports else [""])
        self.port_combo.set(ports[0] if ports else "")

    def _kill_port_owner(self, port: str) -> bool:
        try:
            result = subprocess.run(["lsof", "-t", port],
                                    capture_output=True, text=True)
            pids = result.stdout.strip().split()
            if not pids or pids == ['']:
                return False
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGTERM)
                    self._log(f"Processo {pid} na porta {port} encerrado.")
                except ProcessLookupError:
                    pass
            time.sleep(0.6)
            return True
        except FileNotFoundError:
            return False

    # ─── Conexão ──────────────────────────────────────────────────────────────
    def _toggle_connect(self):
        (self._disconnect if (self.ser and self.ser.is_open)
         else self._connect)()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Erro", "Selecione uma porta serial.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Erro", "Baud rate inválido.")
            return

        if self._kill_port_owner(port):
            self._log("Porta estava ocupada — processo anterior encerrado.")

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self._stop_event.clear()
            self.serial_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self.serial_thread.start()
            self.btn_connect.configure(text="Desconectar",
                                       fg_color="#b71c1c",
                                       hover_color="#c62828")
            th = _DARK if self._dark else _LIGHT
            self.lbl_status.configure(text=f"● {port}",
                                       text_color=th["status_ok"])
            for b in (self.btn_start, self.btn_cal, self.btn_sweep, self.btn_prbs):
                b.configure(state="normal")
            self._log(f"Conectado: {port} @ {baud}")
        except serial.SerialException as exc:
            messagebox.showerror("Erro de Conexão", str(exc))

    def _disconnect(self):
        self._reconnecting = False
        self._stop_event.set()
        self.sweep_running = False
        self.prbs_running  = False
        self.test_active   = False
        self._close_csv_auto()
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.btn_connect.configure(text="Conectar",
                                   fg_color="#0d47a1",
                                   hover_color="#1565c0")
        th = _DARK if self._dark else _LIGHT
        self.lbl_status.configure(text="● Desconectado",
                                   text_color=th["status_default"])
        for b in (self.btn_start, self.btn_cal, self.btn_sweep, self.btn_prbs,
                  self.btn_stop, self.btn_save_csv):
            b.configure(state="disabled")
        self._log("Desconectado.")

    # ─── Reader ───────────────────────────────────────────────────────────────
    def _serial_reader(self):
        while not self._stop_event.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("ascii", errors="ignore").strip()
                    if line:
                        self.root.after(0, self._parse_line, line)
            except serial.SerialException:
                self.root.after(0, self._on_serial_error)
                break
            time.sleep(0.005)

    def _on_serial_error(self):
        if not self._alive:
            return
        self._log("Conexão perdida — tentando reconectar em 3 s...")
        th = _DARK if self._dark else _LIGHT
        self.lbl_status.configure(text="● Reconectando...", text_color="#ffa726")
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        if not self._reconnecting:
            self._reconnecting = True
            self.root.after(3000, self._auto_reconnect)

    def _auto_reconnect(self):
        if self._stop_event.is_set() or not self._alive:
            self._reconnecting = False
            return
        port = self.port_var.get()
        try:
            self._kill_port_owner(port)
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self._stop_event.clear()
            self.serial_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self.serial_thread.start()
            self._reconnecting = False
            th = _DARK if self._dark else _LIGHT
            self.lbl_status.configure(text=f"● {port}",
                                       text_color=th["status_ok"])
            self._log(f"Reconectado: {port}")
            if self.test_active:
                self._send(f"SET:{self.power_var.get():.2f}")
                self._send("START")
                self._log("Ensaio retomado automaticamente.")
        except serial.SerialException:
            self._log("Reconexão falhou — tentando novamente em 3 s...")
            self.root.after(3000, self._auto_reconnect)

    def _periodic_flush(self):
        if not self._alive:
            return
        if self.csv_file:
            try: self.csv_file.flush()
            except Exception: pass
        if self.log_file:
            try: self.log_file.flush()
            except Exception: pass
        self.root.after(5000, self._periodic_flush)

    def _parse_line(self, line: str):
        if not self._alive:
            return
        if line.startswith("WARN:"):
            self.log_box.configure(state="normal")
            self.log_box.insert("end", f"  FIRMWARE: {line}\n", "warn")
            self.log_box.tag_config("warn", foreground="#ef5350")
            self.log_box.see("end")
            self.log_box.configure(state="disabled")
            return
        if not line.startswith("DATA:"):
            self._log(f"← {line}")
            return
        now_raw = time.time()
        if now_raw - self._last_raw_log >= 1.0:
            self._last_raw_log = now_raw
            self._log(f"← {line}")
        parts = line[5:].split(",")
        if len(parts) not in (4, 6):
            return
        try:
            vals = [float(x) for x in parts]
            ms, delay_us, irms, temp = vals[0], vals[1], vals[2], vals[3]
            urms = vals[4] if len(vals) == 6 else None
            prms = vals[5] if len(vals) == 6 else None
        except ValueError:
            return

        if self.t0 == 0.0:
            self.t0 = ms / 1000.0

        t         = ms / 1000.0 - self.t0
        power_pct = delay_us_to_power_pct(delay_us)

        self.times.append(t)
        self.powers.append(power_pct)
        self.currents.append(irms)
        self.temps.append(temp)
        self.voltages.append(urms if urms is not None else float("nan"))
        self.powers_w.append(prms if prms is not None else float("nan"))

        self._label_counter += 1
        if self._label_counter % 5 == 0:
            self.lbl_temp.configure(
                text=f"T       : {temp:.2f} °C" if temp > -100 else "T       : --- °C")
            self.lbl_irms.configure(text=f"I RMS   : {irms:.4f} A")
            self.lbl_pwr_fb.configure(text=f"u(t)    : {power_pct:.1f} %")
            if urms is not None:
                self.lbl_urms.configure(text=f"U RMS   : {urms:.2f} V")
            if prms is not None:
                self.lbl_prms.configure(text=f"P real  : {prms:.2f} W")

        now = time.time()
        if now - self._last_data_log >= 2.0:
            self._last_data_log = now
            temp_str = f"{temp:.2f}°C" if temp > -100 else "---°C"
            u_str = f"  U={urms:.1f}V" if urms is not None else ""
            p_str = f"  P={prms:.1f}W" if prms is not None else ""
            self._log(f"u={power_pct:.1f}%  I={irms:.4f}A  T={temp_str}{u_str}{p_str}")

        self._plot_counter += 1
        if self._plot_counter % 4 == 0:
            self._update_plot()

        if self.csv_writer:
            row = [
                datetime.now().strftime("%H:%M:%S.%f")[:-3],
                f"{ms:.0f}", f"{delay_us:.0f}",
                f"{power_pct:.2f}", f"{irms:.4f}", f"{temp:.2f}",
                f"{urms:.2f}" if urms is not None else "",
                f"{prms:.3f}" if prms is not None else "",
            ]
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def _update_plot(self):
        if not self._alive or not self.times:
            return
        t  = list(self.times)
        vv = list(self.voltages)
        ww = list(self.powers_w)

        self.line_t.set_data(t, list(self.temps))
        self.line_i.set_data(t, list(self.currents))
        self.line_p.set_data(t, list(self.powers))

        # Voltage and power-W only when firmware provides them (not all NaN)
        valid_v = [v for v in vv if not (isinstance(v, float) and v != v)]
        valid_w = [w for w in ww if not (isinstance(w, float) and w != w)]
        if valid_v:
            self.line_v.set_data(t[:len(vv)], vv)
        if valid_w:
            self.line_w.set_data(t[:len(ww)], ww)

        for ax in (self.ax_temp, self.ax_i, self.ax_v, self.ax_p, self.ax_w):
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    # ─── Linearização ────────────────────────────────────────────────────────
    def _carregar_lut(self):
        path = filedialog.askopenfilename(
            title="Selecione a LUT de linearização",
            initialdir=os.path.dirname(os.path.abspath(__file__)),
            filetypes=[("CSV", "*.csv"), ("Todos", "*.*")])
        if not path:
            return
        try:
            import csv as csv_mod
            I_vals, pct_vals = [], []
            with open(path, newline="") as f:
                reader = csv_mod.DictReader(f)
                for row in reader:
                    I_vals.append(float(row["irms_A_desejado"]))
                    pct_vals.append(float(row["potencia_pct_a_enviar"]))
            self._lut_I   = np.array(I_vals)
            self._lut_pct = np.array(pct_vals)
            nome = os.path.basename(path)
            self.lbl_lut.configure(
                text=f"LUT: {nome}\n{len(I_vals)} pts  "
                     f"I=[{I_vals[0]:.3f}–{I_vals[-1]:.3f}]A",
                text_color="#2e7d32")
            self.linearizar_ativo.set(True)
            self._log(f"LUT carregada: {nome} ({len(I_vals)} pontos)")
        except Exception as e:
            messagebox.showerror("Erro", f"Não foi possível carregar a LUT:\n{e}")

    def _aplicar_lut(self, pct: float) -> float:
        if not self.linearizar_ativo.get() or self._lut_I is None:
            return pct
        I_min  = float(self._lut_I[0])
        I_max  = float(self._lut_I[-1])
        I_alvo = I_min + (pct / 100.0) * (I_max - I_min)
        return float(np.interp(I_alvo, self._lut_I, self._lut_pct))

    # ─── Potência ────────────────────────────────────────────────────────────
    def _on_power_change(self, _=None):
        pwr = self.power_var.get()
        self._lbl_pwr_big.configure(text=f"{pwr:.1f} %")
        if self.test_active:
            self._send(f"SET:{self._aplicar_lut(pwr):.2f}")

    # ─── Ensaio ──────────────────────────────────────────────────────────────
    def _start_test(self, tipo: str = "manual"):
        if not (self.ser and self.ser.is_open):
            return
        self.t0 = 0.0
        for d in (self.times, self.powers, self.currents, self.temps,
                  self.voltages, self.powers_w):
            d.clear()
        self._open_csv_auto(tipo)
        self._send(f"SET:{self.power_var.get():.2f}")
        self._send("START")
        self.test_active = True
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.btn_save_csv.configure(state="normal")
        th = _DARK if self._dark else _LIGHT
        self.lbl_status.configure(text="● Ensaio em andamento",
                                   text_color=th["status_ok"])

    def _stop_test(self):
        if self.ser and self.ser.is_open:
            self._send("STOP")
        self.test_active   = False
        self.sweep_running = False
        self.prbs_running  = False
        self._close_csv_auto()
        if self.ser and self.ser.is_open:
            self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")
        self.btn_sweep.configure(text="Iniciar varredura")
        self.btn_prbs.configure(text="Iniciar PRBS")
        self.lbl_status.configure(text="● Parado", text_color="#ffa726")

    def _reset_geral(self):
        self.sweep_running = False
        self.prbs_running  = False
        if self.ser and self.ser.is_open:
            self._send("STOP")
        self.test_active = False
        self._close_csv_auto()
        for d in (self.times, self.powers, self.currents, self.temps,
                  self.voltages, self.powers_w):
            d.clear()
        self.t0 = 0.0
        self.line_t.set_data([], [])
        self.line_i.set_data([], [])
        self.line_v.set_data([], [])
        self.line_p.set_data([], [])
        self.line_w.set_data([], [])
        if self._alive:
            self.canvas.draw_idle()
        self.lbl_temp.configure(  text="T       : --- °C")
        self.lbl_irms.configure(  text="I RMS   : --- A")
        self.lbl_urms.configure(  text="U RMS   : --- V")
        self.lbl_pwr_fb.configure(text="u(t)    : --- %")
        self.lbl_prms.configure(  text="P real  : --- W")
        if self.ser and self.ser.is_open:
            self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")
        self.btn_sweep.configure(text="Iniciar varredura")
        self.btn_prbs.configure( text="Iniciar PRBS")
        self.lbl_status.configure(text="● Pronto", text_color="#ffa726")
        self._log("Reset geral executado.")

    def _calibrate(self):
        self._send("CAL")
        self._log("Calibrando... retire a carga antes de confirmar.")

    def _send(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
                self._log(f"→ {cmd}")
            except serial.SerialException as exc:
                self._log(f"Erro serial: {exc}")

    # ─── CSV ─────────────────────────────────────────────────────────────────
    def _open_csv_auto(self, tipo: str = "manual"):
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        pasta    = os.path.dirname(os.path.abspath(__file__))
        base     = os.path.join(pasta, f"ensaio_{tipo}_{ts}")
        csv_path = base + ".csv"
        log_path = base + "_log.txt"
        self.csv_file   = open(csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["timestamp", "millis", "delay_us",
             "potencia_pct", "irms_A", "temp_C",
             "urms_V", "potencia_W"])
        self.log_file = open(log_path, "w", encoding="utf-8")
        self.log_file.write(f"=== Sessão iniciada: {datetime.now()} ===\n")
        self._log(f"CSV: {csv_path}")
        self._log(f"LOG: {log_path}")

    def _close_csv_auto(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.csv_writer = None
            self._log("CSV fechado.")
        if self.log_file:
            self.log_file.write(f"=== Sessão encerrada: {datetime.now()} ===\n")
            self.log_file.close()
            self.log_file = None

    def _export_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Texto", "*.txt"), ("Todos", "*.*")],
            initialfile=f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if not path:
            return
        self.log_box.configure(state="normal")
        content = self.log_box.get("1.0", "end")
        self.log_box.configure(state="disabled")
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        self._log(f"Log exportado: {path}")

    def _export_csv(self):
        if not self.currents:
            messagebox.showinfo("Aviso", "Nenhum dado ainda.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Todos", "*.*")],
            initialfile=f"ensaio_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["tempo_s", "potencia_pct", "irms_A", "temp_C"])
            for row in zip(self.times, self.powers, self.currents, self.temps):
                w.writerow([f"{v:.4f}" for v in row])
        self._log(f"Exportado: {path}")
        messagebox.showinfo("Pronto", f"Salvo em:\n{path}")

    # ─── Varredura ───────────────────────────────────────────────────────────
    def _toggle_sweep(self):
        if self.sweep_running:
            self.sweep_running = False
            self.btn_sweep.configure(text="Iniciar varredura")
        else:
            self._run_sweep()

    def _run_sweep(self):
        try:
            p_from  = float(self.sweep_from_var.get())
            p_to    = float(self.sweep_to_var.get())
            p_step  = float(self.sweep_step_var.get())
            p_dwell = float(self.sweep_dwell_var.get())
        except ValueError:
            messagebox.showerror("Erro", "Parâmetros inválidos.")
            return
        if p_step <= 0:
            messagebox.showerror("Erro", "Passo deve ser positivo.")
            return
        if not self.test_active:
            self._start_test("varredura")
        self.sweep_running = True
        self.btn_sweep.configure(text="Parar varredura")

        def worker():
            for pwr in np.arange(p_from, p_to + p_step / 2.0, p_step):
                if not self.sweep_running:
                    break
                pwr = float(np.clip(pwr, 0.0, 100.0))
                self.root.after(0, self.power_var.set, pwr)
                self.root.after(0, lambda p=pwr: self._lbl_pwr_big.configure(
                    text=f"{p:.1f} %"))
                self._send(f"SET:{self._aplicar_lut(pwr):.2f}")
                self._log(f"Varredura → {pwr:.1f} %")
                time.sleep(p_dwell)
            self.sweep_running = False
            self.root.after(0, self.btn_sweep.configure,
                            {"text": "Iniciar varredura"})
            self._log("Varredura concluída.")
            self.root.after(0, self._stop_test)

        self.sweep_thread = threading.Thread(target=worker, daemon=True)
        self.sweep_thread.start()

    # ─── Log ─────────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        if not self._alive:
            return
        if threading.current_thread() is not threading.main_thread():
            self.root.after(0, self._log, msg)
            return
        ts   = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}]  {msg}\n"
        self.log_box.configure(state="normal")
        self.log_box.insert("end", line)
        self.log_box.see("end")
        self.log_box.configure(state="disabled")
        if self.log_file:
            self.log_file.write(line)

    # ─── PRBS ────────────────────────────────────────────────────────────────
    @staticmethod
    def _gen_prbs_seq(n_bits: int) -> np.ndarray:
        TAPS  = {5: (5, 2), 6: (6, 5), 7: (7, 6)}
        tap_a, tap_b = TAPS[n_bits]
        mask  = (1 << n_bits) - 1
        state = mask
        seq   = []
        for _ in range(mask):
            fb    = ((state >> (tap_a - 1)) ^ (state >> (tap_b - 1))) & 1
            state = ((state << 1) | fb) & mask
            seq.append(state & 1)
        return np.array(seq, dtype=int)

    def _toggle_prbs(self):
        if self.prbs_running:
            self.prbs_running = False
            self.btn_prbs.configure(text="Iniciar PRBS")
        else:
            self._run_prbs()

    def _run_prbs(self):
        try:
            u_min    = float(self.prbs_umin_var.get())
            u_max    = float(self.prbs_umax_var.get())
            interval = float(self.prbs_interval_var.get())
            n_bits   = int(self.prbs_bits_var.get()[0])
        except ValueError:
            messagebox.showerror("Erro", "Parâmetros PRBS inválidos.")
            return
        if u_min >= u_max:
            messagebox.showerror("Erro", "u_min deve ser menor que u_max.")
            return
        if not self.test_active:
            self._start_test("prbs")

        seq = self._gen_prbs_seq(n_bits)
        period = len(seq)
        self.prbs_running = True
        self.btn_prbs.configure(text="Parar PRBS")
        self._log(f"PRBS: {period} amostras × {interval:.0f}s "
                  f"= {period * interval / 60:.1f} min")

        def worker():
            for bit in seq:
                if not self.prbs_running:
                    break
                pwr = u_max if bit else u_min
                self.root.after(0, self.power_var.set, pwr)
                self.root.after(0, lambda p=pwr: self._lbl_pwr_big.configure(
                    text=f"{p:.1f} %"))
                self._send(f"SET:{self._aplicar_lut(pwr):.2f}")
                self._log(f"PRBS bit={bit} → {pwr:.0f}%")
                time.sleep(interval)
            self.prbs_running = False
            self.root.after(0, self.btn_prbs.configure,
                            {"text": "Iniciar PRBS"})
            self._log("PRBS concluído.")

        self.prbs_thread = threading.Thread(target=worker, daemon=True)
        self.prbs_thread.start()

    # ─── Fechamento ──────────────────────────────────────────────────────────
    def _on_close(self):
        self._alive = False
        self.sweep_running = False
        self.prbs_running  = False
        self._stop_event.set()
        self._close_csv_auto()
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"STOP\n")
                self.ser.close()
            except Exception:
                pass
        try:
            self.fig.clear()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = ctk.CTk()
    EnsaioApp(root)
    root.mainloop()
