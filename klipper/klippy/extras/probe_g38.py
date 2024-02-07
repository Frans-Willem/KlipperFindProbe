import math
import inspect

# Singleton class, will be registered under object "probe_g38_global"
class G38Commands():
    def get(printer):
        name = 'probe_g38_commands'
        instance = printer.lookup_object(name, default = None)
        if instance is None:
            instance = G38Commands(printer)
            printer.add_object(name, instance)
        return instance

    def __init__(self, printer):
        self.printer = printer
        self.probes = dict()
        self.selected = "default"

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("QUERY_G38", self.cmd_QUERY_G38, desc=self.cmd_QUERY_G38_help)
        gcode.register_command("SELECT_G38", self.cmd_SELECT_G38, desc=self.cmd_SELECT_G38_help)
        gcode.register_command("G38.2", self.cmd_G38_2, self.cmd_G38_2_help)
        gcode.register_command("G38.3", self.cmd_G38_3, self.cmd_G38_3_help)
        gcode.register_command("G38.4", self.cmd_G38_4, self.cmd_G38_4_help)
        gcode.register_command("G38.5", self.cmd_G38_5, self.cmd_G38_5_help)

    def register_probe(self, probe):
        self.probes[probe.name] = probe

    def get_probe(self, probe_name):
        return self.probes.get(probe_name, None)

    def get_selected_probe(self):
        probe = self.get_probe(self.selected)
        if probe is None:
            probe = gcode_move.get_probe("default")
        return probe

    cmd_QUERY_G38_help = "Query all registered G38 probes"
    def cmd_QUERY_G38(self, gcmd):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        msg = " ".join("%s:%s" % (name, "TRIGGERED" if probe.query_state(print_time) else "open") for (name, probe) in self.probes.items())
        gcmd.respond_raw(msg)

    cmd_SELECT_G38_help = "Select probe to be used in subsequent G38.x commands"
    def cmd_SELECT_G38(self, gcmd):
        name = gcmd.get('NAME', "default")
        if name not in self.probes:
            raise gcmd.error("No probe with name '%s' found" % (name))
            return
        self.selected = name
        gcmd.respond_info("G38 probe '%s' selected" % (name))


    def cmd_G38_generic(self, gcmd, stop_on, raise_error):
        probe = self.get_selected_probe()
        return probe.probe_to_gcode_position(gcmd, stop_on=stop_on, raise_error=raise_error)

    cmd_G38_2_help = "probe toward workpiece, stop on contact, signal error if failure"
    def cmd_G38_2(self, gcmd):
        return self.cmd_G38_generic(gcmd, stop_on=True, raise_error=True)

    cmd_G38_3_help = "probe toward workpiece, stop of contact"
    def cmd_G38_3(self, gcmd):
        return self.cmd_G38_generic(gcmd, stop_on=True, raise_error=False)

    cmd_G38_4_help = "probe away from workpiece, stop on loss of contact, signal error if failure"
    def cmd_G38_4(self, gcmd):
        return self.cmd_G38_generic(gcmd, stop_on=False, raise_error=True)

    cmd_G38_5_help = "probe away from workpiece, stop on loss of contact, signal error if failure"
    def cmd_G38_5(self, gcmd):
        return self.cmd_G38_generic(gcmd, stop_on=False, raise_error=False)


class G38Probe():
    def __init__(self, config):
        self.printer = config.get_printer()
        config_names = config.get_name().split()
        self.name = config_names[1] if len(config_names)>1 else "default"
        self.commands = G38Commands.get(self.printer) 
        pins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = pins.lookup_pin(pin, can_invert=True, can_pullup=True)
        self.endstop = pin_params['chip'].setup_pin('endstop', pin_params)
        self.commands.register_probe(self)

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        for stepper in self.printer.lookup_object('toolhead').get_kinematics().get_steppers():
            self.endstop.add_stepper(stepper)


    def query_state(self, print_time):
        return self.endstop.query_endstop(print_time)

    def probe_to_gcode_position(self, gcmd, gcode_position=None, gcode_speed=None, stop_on=True, raise_error=True):
        # G38 should use gcode_move positions,
        # and as there's no documented way to go from GCode position to toolhead position taking into account everything like skew correction,
        # we have to dirtily hijack the toolhead, simulate a G1, and intercept the target and speed
        # You can either pass in a gcode position in gcode_position, or (when None) in X,Y,Z,F parameters in gcmd
        toolhead = self.printer.lookup_object('toolhead')
        gcode_move = self.printer.lookup_object('gcode_move')

        if gcode_speed is None:
            gcode_speed = gcode_move.speed

        # TODO: Convert this into something of a with ...: python syntax ?
        with G38LookaheadIntercept(toolhead) as intercept:
            if gcode_position is None:
                gcode_move.cmd_G1(gcmd)
            else:
                gcode_move.move_with_transform(gcode_position, gcode_speed)

            toolhead_position = toolhead.commanded_pos
            last_move = intercept.get_last()
            if last_move is None:
                toolhead_speed = math.sqrt(last_move.max_cruise_v2)
            else:
                toolhead_speed = gcode_speed

        return self.probe_to_toolhead_position(gcmd, toolhead_position, toolhead_speed, stop_on, raise_error)

    def probe_to_toolhead_position(self, gcmd, target_position, target_speed, stop_on, raise_error):
        gcmd.respond_info("Probing to (%f,%f,%f) at %fmm/sec" % (target_position[0], target_position[1], target_position[2], target_speed))

        toolhead = self.printer.lookup_object('toolhead')
        homing = self.printer.lookup_object('homing')
        homing_mod = inspect.getmodule(homing)

        hmove = homing_mod.HomingMove(self.printer, [(self.endstop, self.name)], toolhead)
        try:
            epos = hmove.homing_move(target_position, target_speed, probe_pos=True, triggered=stop_on,
                              check_triggered=raise_error)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown")
            raise

        gcmd.respond_info("Final position (%f,%f,%f)" % (epos[0], epos[1], epos[2]))
        return epos



class G38LookaheadIntercept():
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = [] 
        self._entered = False
        self._old_lookahead = None
        self._old_commanded_pos = None

    def __enter__(self):
        if self._entered:
            raise Exception("G38LookaheadIntercept __enter__ called twice")
        self._entered = True
        self._old_lookahead = self.toolhead.lookahead
        self._old_commanded_pos = self.toolhead.commanded_pos
        self.toolhead.lookahead = self
        self.toolhead.commanded_pos = list(self._old_commanded_pos)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if not self._entered:
            raise Exception("G38LookaheadIntercept __exit__ called without matching __enter__")
        self.toolhead.lookahead = self._old_lookahead
        self.toolhead.commanded_pos = self._old_commanded_pos
        self._entered = False
    def reset(self):
        del self.queue[:]
    def set_flush_time(self, flush_time):
        return
    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None
    def flush(self, lazy=False):
        return
    def add_move(self, move):
        self.queue.append(move)


def load_config(config):
    return G38Probe(config)

def load_config_prefix(config):
    return G38Probe(config)

