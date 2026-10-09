"""Black surfaces, white typography and neon green control outlines."""

from tkinter import ttk

BACKGROUND = "#070b0a"
SURFACE = "#101816"
INPUT = "#080f0d"
NEON = "#39ff9a"
WHITE = "#f5fff9"
MUTED = "#bbcfc3"
BORDER = "#325346"
ERROR = "#ff9e9e"


def apply_theme(root) -> ttk.Style:
    root.configure(background=BACKGROUND)
    root.option_add("*TCombobox*Listbox.background", INPUT)
    root.option_add("*TCombobox*Listbox.foreground", WHITE)
    root.option_add("*TCombobox*Listbox.selectBackground", "#164c32")
    root.option_add("*TCombobox*Listbox.selectForeground", WHITE)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=SURFACE, foreground=WHITE, font=("Segoe UI", 10))
    style.configure("TFrame", background=SURFACE)
    style.configure("Page.TFrame", background=BACKGROUND)
    style.configure("TNotebook", background=BACKGROUND, borderwidth=0)
    style.configure("TNotebook.Tab", background=SURFACE, foreground=WHITE, padding=(18, 9), font=("Segoe UI", 11, "bold"))
    style.map("TNotebook.Tab", background=[("selected", "#125c37"), ("active", "#164c32")], foreground=[("selected", WHITE), ("!selected", MUTED)])
    style.configure("TLabel", background=SURFACE, foreground=WHITE)
    style.configure("Muted.TLabel", foreground=MUTED)
    style.configure("Page.TLabel", background=BACKGROUND, foreground=WHITE)
    style.configure("PageMuted.TLabel", background=BACKGROUND, foreground=MUTED)
    style.configure("Brand.TLabel", background=BACKGROUND, foreground=NEON, font=("Segoe UI", 24, "bold"))
    style.configure("Section.TLabel", foreground=WHITE, font=("Segoe UI", 11, "bold"))
    style.configure("TRadiobutton", background=SURFACE, foreground=WHITE, indicatorcolor=INPUT, focuscolor=NEON)
    style.map("TRadiobutton", background=[("active", SURFACE)], foreground=[("!disabled", WHITE)], indicatorcolor=[("selected", NEON), ("!selected", INPUT)])
    style.configure("TLabelframe", background=SURFACE, bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER, borderwidth=1, relief="solid")
    style.configure("TLabelframe.Label", background=SURFACE, foreground=NEON, font=("Segoe UI", 13, "bold"))
    style.configure("Motor.TLabelframe", bordercolor=NEON, lightcolor=NEON, darkcolor=NEON, borderwidth=1)
    style.configure("Metric.TFrame", background=INPUT)
    style.configure("Metric.TLabel", background=INPUT, foreground=MUTED, font=("Segoe UI", 9, "bold"))
    style.configure("MetricValue.TLabel", background=INPUT, foreground=WHITE, font=("Consolas", 23, "bold"))
    style.configure("Badge.TLabel", background="#1b2a23", foreground=WHITE, padding=(12, 6), font=("Segoe UI", 10, "bold"))
    style.configure("Enabled.Badge.TLabel", background="#125c37", foreground=WHITE)
    style.configure("Error.Badge.TLabel", background="#442021", foreground=WHITE)
    style.configure("Hex.TLabel", background=INPUT, foreground=WHITE, padding=(10, 8), font=("Consolas", 10))
    style.configure("TButton", background=INPUT, foreground=WHITE, bordercolor=NEON, lightcolor=NEON, darkcolor=NEON, borderwidth=1, relief="solid", padding=(10, 7), font=("Segoe UI", 10, "bold"), focusthickness=1, focuscolor=NEON)
    style.map("TButton", background=[("disabled", "#101713"), ("pressed", "#165836"), ("active", "#123e2a")], foreground=[("disabled", "#b1bdb5"), ("!disabled", WHITE)], bordercolor=[("disabled", BORDER), ("!disabled", NEON)], lightcolor=[("disabled", BORDER), ("!disabled", NEON)], darkcolor=[("disabled", BORDER), ("!disabled", NEON)])
    style.configure("Primary.TButton", background="#125c37", borderwidth=2)
    style.map("Primary.TButton", background=[("disabled", "#101713"), ("pressed", "#19754a"), ("active", "#177549"), ("!disabled", "#125c37")])
    style.configure("Stop.TButton", borderwidth=2, padding=(16, 9))
    for name in ("TEntry", "TCombobox"):
        style.configure(name, fieldbackground=INPUT, background=INPUT, foreground=WHITE, insertcolor=WHITE, bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER, borderwidth=1, padding=7, arrowcolor=NEON, selectbackground="#164c32", selectforeground=WHITE)
        style.map(name, fieldbackground=[("disabled", "#17221c"), ("readonly", INPUT), ("!disabled", INPUT)], foreground=[("disabled", MUTED), ("!disabled", WHITE)], bordercolor=[("focus", NEON), ("!focus", BORDER)], background=[("active", "#164c32"), ("!active", INPUT)])
    style.configure("Vertical.TScrollbar", background="#224c36", troughcolor=BACKGROUND, bordercolor=BACKGROUND, arrowcolor=NEON, lightcolor=BACKGROUND, darkcolor=BACKGROUND)
    style.map("Vertical.TScrollbar", background=[("active", "#328557"), ("!active", "#224c36")])
    style.configure("Treeview", background=INPUT, fieldbackground=INPUT, foreground=WHITE, rowheight=30, bordercolor=BORDER, font=("Segoe UI", 10))
    style.configure("Treeview.Heading", background="#152e21", foreground=WHITE, font=("Segoe UI", 10, "bold"), relief="flat")
    style.map("Treeview", background=[("selected", "#164c32")], foreground=[("selected", WHITE)])
    return style
