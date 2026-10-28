"""Scrollable, input-aligned force comparisons for every ramp condition."""
import numpy as np
from PyQt6.QtWidgets import QTabWidget, QScrollArea, QWidget, QVBoxLayout, QComboBox
from matplotlib import colormaps
from .plot_panel import PlotPanel


def event_identity(event):
    return (id(event.run), event.start, event.stop, event.baseline_stop, event.axis)


class TransientPanel(QTabWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.panels = {}
        self.locations = {}
        self.replays = {}
        self.group_pages = {}
        for component, label in (('fy', 'Lateral force — Fy'),):
            container = QWidget()
            layout = QVBoxLayout(container)
            selector = QComboBox()
            layout.addWidget(selector)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            panel = PlotPanel(count=1)
            scroll.setWidget(panel)
            layout.addWidget(scroll)
            self.addTab(container, label)
            self.panels[component] = panel
            self.group_pages[component] = selector
            selector.currentIndexChanged.connect(lambda index, comp=component: self._show_page(comp, index))
        self.message('Load TTC runs to compare isolated, zero-IA step-steer events.')

    def message(self, text):
        for panel in self.panels.values():
            panel.message(text)

    def show_groups(self, groups):
        new_groups = tuple(groups)
        identities = tuple(tuple(event_identity(e) for e in g.events) for g in new_groups)
        old_identities = getattr(self, '_group_identities', None)
        if old_identities != identities:
            self.replays.clear()
        self._groups = new_groups
        self._group_identities = identities
        self.locations = {}
        for component, selector in self.group_pages.items():
            old_page = max(0, selector.currentIndex())
            selector.blockSignals(True)
            selector.clear()
            selector.addItems([f'Conditions {start + 1}–{min(start + 8, len(self._groups))} of {len(self._groups)}'
                                for start in range(0, len(self._groups), 8)])
            selector.setEnabled(bool(self._groups))
            selector.setCurrentIndex(min(old_page, max(0, selector.count() - 1)))
            selector.blockSignals(False)
            self._show_page(component, selector.currentIndex())

    def _show_page(self, component, page):
        from ..transient_plotting import aligned_time
        panel = self.panels[component]
        self.locations = {identity: [loc for loc in locs if loc[3] != component]
                          for identity, locs in self.locations.items()}
        self.locations = {identity: locs for identity, locs in self.locations.items() if locs}
        panel.figure.clear()
        first = max(0, page) * 8
        groups = self._groups[first:first + 8]
        count = max(1, len(groups))
        columns = min(2, count)
        rows = (count + columns - 1) // columns
        panel.figure.set_size_inches(12, max(4, rows * 3.5))
        panel.canvas.setMinimumHeight(max(350, rows * 350))
        all_axes = panel.figure.subplots(rows, columns, squeeze=False).ravel()
        panel.axes = all_axes[:max(1, len(groups))]
        for unused in all_axes[max(1, len(groups)): ]:
            panel.figure.delaxes(unused)
        if not groups:
            panel.message('No isolated zero-IA step-steer events detected. Continuous sweeps are excluded.')
            return
        for offset, (axis, group) in enumerate(zip(panel.axes, groups)):
            command_axis = axis.twinx()
            axis._command_axis = command_axis
            loads = sorted(set(round(float(np.median(e.run.channels['fz'][e.start:e.stop])) / 50) * 50 for e in group.events))
            colors = {load: colormaps['viridis'](i / max(1, len(loads)-1)) for i, load in enumerate(loads)}
            for event in group.events:
                c = event.run.channels
                sl = slice(event.start, event.stop)
                t = aligned_time(event)
                load = round(float(np.median(c['fz'][sl])) / 50) * 50
                color = colors[load]
                label = f'Fz ≈ {load:g} N'
                axis.plot(t, c[component][sl], color=color, alpha=.65, label=label + ' measured')
                command = np.rad2deg(c['alpha'][sl])
                # One representative command per group; alignment still uses each
                # event's exact recorded SA, not this display reference.
                if event is group.events[0]:
                    command_axis.plot(t, command, color='#888888', linestyle=':',
                                      alpha=.6, linewidth=1, label='SA command')
                if event.axis == 'longitudinal':
                    command_axis.plot(t, c['kappa'][sl] * 100, color=color, linestyle='-.', alpha=.5,
                                      label=label + ' slip command')
                self.locations.setdefault(event_identity(event), []).append((axis, t, color, component, label))
            title = (f'{group.belt_speed:.2f} m/s · ramp {group.ramp_time:.2f} s · '
                     f'{group.steer_rate:.2f} {"deg/s" if group.axis == "lateral" else "1/s"} · {group.direction:+d}')
            if group.tire_id:
                title += f' · {group.tire_id}'
            title += f' · IA {np.rad2deg(group.inclination):.2f}° · P {group.pressure/1000:.1f} kPa'
            if group.axis == 'longitudinal':
                title += f' · SA {np.rad2deg(group.secondary_slip):.2f}°'
            axis.set_title(title)
            axis.set_xlabel('Time from SA midpoint (s)' if group.axis == 'lateral'
                            else 'Time from slip-ratio midpoint (s)')
            axis.set_ylabel(component.upper() + ' (N)')
            command_axis.set_ylabel('SA command (deg)' + (' / slip command (%)' if group.axis == 'longitudinal' else ''))
            # Keep legends about the two force groups, not every input reference.
            axis.grid(True, alpha=.25)
            handles, labels = axis.get_legend_handles_labels()
            command_handles, command_labels = command_axis.get_legend_handles_labels()
            axis.legend(handles + command_handles, labels + command_labels, fontsize='xx-small', loc='best')
        panel.paletteChanged()
        panel.figure.tight_layout()
        self._draw_page_replays(component)
        panel.canvas.draw_idle()

    def add_replay(self, event, predictions):
        self.replays[event_identity(event)] = predictions
        self._draw_replay(event)

    def _draw_replay(self, event):
        identity = event_identity(event)
        if identity not in self.replays:
            return
        current, seed = self.replays[identity]
        for axis, t, color, component, label in self.locations.get(event_identity(event), ()):
            column = 0 if component == 'fx' else 1
            for line in list(axis.lines):
                if line.get_gid() == str(event_identity(event)):
                    line.remove()
            for values, style, name in ((current, '--', 'current model'),):
                if values is not None:
                    line, = axis.plot(t, values[:, column], color=color, linestyle=style,
                                      label=label + ' ' + name)
                    line.set_gid(str(event_identity(event)))
            handles, labels = axis.get_legend_handles_labels()
            command_axis = axis._command_axis
            command_handles, command_labels = command_axis.get_legend_handles_labels()
            axis.legend(handles + command_handles, labels + command_labels,
                        fontsize='xx-small', loc='best')
        for panel in self.panels.values():
            panel.canvas.draw_idle()

    def _draw_page_replays(self, component):
        panel = self.panels[component]
        visible = {event_identity(event) for group in self._groups[
            max(0, self.group_pages[component].currentIndex()) * 8:
            max(0, self.group_pages[component].currentIndex()) * 8 + 8]
                   for event in group.events}
        for identity in visible.intersection(self.replays):
            event = next(event for group in self._groups for event in group.events
                         if event_identity(event) == identity)
            self._draw_replay(event)
        panel.canvas.draw_idle()
