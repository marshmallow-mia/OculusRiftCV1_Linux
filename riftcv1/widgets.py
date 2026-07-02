"""Shared GTK widgets."""
import gi
gi.require_version("Gtk", "4.0")
from gi.repository import Gtk


class StatusRow(Gtk.Box):
    """Compact one-line status row: state icon, name, value."""
    ICONS = {True: ("emblem-ok-symbolic", "success"),
             False: ("dialog-error-symbolic", "error"),
             None: ("dialog-warning-symbolic", "warning")}

    def __init__(self, label):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.set_margin_top(3)
        self.set_margin_bottom(3)
        self.set_margin_start(10)
        self.set_margin_end(10)
        self.icon = Gtk.Image.new_from_icon_name("content-loading-symbolic")
        self.name = Gtk.Label(label=label, xalign=0, hexpand=True)
        self.value = Gtk.Label(label="…", xalign=1)
        self.value.add_css_class("dim-label")
        self.value.set_wrap(True)
        self.append(self.icon)
        self.append(self.name)
        self.append(self.value)

    def set(self, ok, text):
        name, cls = self.ICONS[ok]
        self.icon.set_from_icon_name(name)
        for c in ("success", "error", "warning"):
            self.icon.remove_css_class(c)
        self.icon.add_css_class(cls)
        self.value.set_text(text)
