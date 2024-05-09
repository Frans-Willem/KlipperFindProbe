import math

class FindProbe():
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.name = config.get_name()
        # Travel speed in X,Y dimension
        self.travel_speed = config.getfloat("travel_speed", 10.0)
        # Travel speed in Z dimension (e.g. lift)
        self.z_travel_speed = config.getfloat('z_travel_speed', 5.0)
        # Probe speed in X,Y dimension: Speed at which X or Y are probed
        self.probe_speed = config.getfloat('probe_speed', 5.0)
        # Probe speed in Z dimension
        self.z_probe_speed = config.getfloat('z_probe_speed', 5.0)
        # Angles at which X,Y probing is done. Note that the opposite angle will also be probed, such that both sides of the probe can be averaged.
        # e.g. if you specify both 0 and 180, the Y dimension will be probed twice.
        self.angles = config.getfloatlist('angles', [0,90,180,270])
        # Maximum to drop past current probe height estimate for Z probing
        # Can be set to 0 to disable Z probing.
        self.z_probe_plunge = config.getfloat('z_probe_plunge', 5.0)
        # How far to drop past current probe height estimate for X/Y probing
        self.xy_probe_plunge = config.getfloat('xy_probe_plunge', 0.5)
        # How far out to start for XY probes
        self.spread = config.getfloat('spread', 7.0)
        # How far above current probe estimate we should lift for travel moves
        self.lift = config.getfloat('lift', 3.0)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("FIND_PROBE", self.cmd_FIND_PROBE, self.cmd_FIND_PROBE_help)


        return

    # Lifts, moves, and probes, returns where probe hit.
    # In the future, this may also do multi-sampling
    def _probe(self, gcmd, probe, estimated_probe_pos, source_pos, target_pos, probe_speed):
        gcmd.respond_info("Probe source: (%f,%f,%f)" % (source_pos[0], source_pos[1], source_pos[2]))
        gcmd.respond_info("Probe target: (%f,%f,%f)" % (target_pos[0], target_pos[1], target_pos[2]))
        gcode_move = self.gcode_move

        # Lift first
        lifted_pos = list(gcode_move.last_position)
        lifted_pos[2] = estimated_probe_pos[2] + self.lift
        gcode_move.move_with_transform(lifted_pos, self.z_travel_speed)

        # Move above source position
        lifted_source_pos = list(source_pos)
        lifted_source_pos[2] = lifted_pos[2]
        gcode_move.move_with_transform(lifted_source_pos, self.travel_speed)

        # Drop to source position
        gcode_move.move_with_transform(source_pos, self.z_travel_speed)

        # TODO: Move toolhead waiting and pre- and post-stepper positions to probe code
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        pre_stepper_positions = [s.get_mcu_position() for s in toolhead.get_kinematics().get_steppers()]
        gcmd.respond_info("Pre stepper positions: %s" % (str(pre_stepper_positions)))
        probe_position = probe.probe_to_gcode_position(gcmd, gcode_position=target_pos, gcode_speed=probe_speed)
        post_stepper_positions = [s.get_mcu_position() for s in toolhead.get_kinematics().get_steppers()]
        gcmd.respond_info("Post stepper positions: %s" % (str(post_stepper_positions)))
        if pre_stepper_positions == post_stepper_positions:
            raise gcmd.error("Probe triggered prior to movement")

        # TODO: Proper retract?
        gcode_move.move_with_transform(source_pos, probe_speed)
        return probe_position


    # Lifts, moves, probes Z, returns where it hit
    def _probe_z(self, gcmd, probe, estimated_probe_pos):
        source_pos = list(estimated_probe_pos)
        target_pos = list(estimated_probe_pos)
        source_pos[2] += self.lift
        target_pos[2] -= self.z_probe_plunge
        return self._probe(gcmd, probe, estimated_probe_pos, source_pos, target_pos, self.z_probe_speed)

    def _probe_xy(self, gcmd, probe, estimated_probe_pos, angle):
        results = []
        for direction in [1, -1]:
            angle_rad = math.radians(angle)
            dx = math.sin(angle_rad) * self.spread * direction
            dy = math.cos(angle_rad) * self.spread * direction
            source_pos = list(estimated_probe_pos)
            target_pos = list(estimated_probe_pos)
            source_pos[2] -= self.xy_probe_plunge
            target_pos[2] -= self.xy_probe_plunge
            source_pos[0] += dx
            source_pos[1] += dy
            target_pos[0] -= dx
            target_pos[1] -= dy
            results.append(self._probe(gcmd, probe, estimated_probe_pos, source_pos, target_pos, self.probe_speed))
        # Average
        first = results[0]
        second = results[1]
        for i in range(len(first)):
            first[i] = (first[i] + second[i]) / 2.0
        # Overwrite Z-value
        first[2] = estimated_probe_pos[2]
        return first

    cmd_FIND_PROBE_help = "Find probe"
    def cmd_FIND_PROBE(self, gcmd):
        probe_g38_commands = self.printer.lookup_object('probe_g38_commands')
        if  probe_g38_commands is None:
            raise gcmd.error("Please define at least one probe_g38 section")
        probe = probe_g38_commands.get_selected_probe()
        if  probe is None:
            raise gcmd.error("Please define at least one probe_g38 section")

        gcode_move = self.gcode_move


        # Start off with current position as possible probe position
        estimated_probe_pos = list(gcode_move.last_position)

        # Adjust with G-Code parameters
        params = gcmd.get_command_parameters()
        for pos, axis in enumerate('XYZ'):
            if axis in params:
                v = float(params[axis])
                if not gcode_move.absolute_coord:
                    estimated_probe_pos[pos] += v
                else:
                    estimated_probe_pos[pos] = v + gcode_move.base_position[pos]

        gcmd.respond_info("Estimated position: (%f,%f,%f)" % (estimated_probe_pos[0], estimated_probe_pos[1], estimated_probe_pos[2]))

        if self.z_probe_plunge > 0.0:
            estimated_probe_pos = self._probe_z(gcmd, probe, estimated_probe_pos)
            gcmd.respond_info("Z Probe adjustment: (%f,%f,%f)" % (estimated_probe_pos[0], estimated_probe_pos[1], estimated_probe_pos[2]))

        for angle in self.angles:
            estimated_probe_pos = self._probe_xy(gcmd, probe, estimated_probe_pos, angle)
            gcmd.respond_info("XY Probe adjustment: (%f,%f,%f)" % (estimated_probe_pos[0], estimated_probe_pos[1], estimated_probe_pos[2]))
            if self.z_probe_plunge > 0.0:
                estimated_probe_pos = self._probe_z(gcmd, probe, estimated_probe_pos)
                gcmd.respond_info("Z Probe adjustment: (%f,%f,%f)" % (estimated_probe_pos[0], estimated_probe_pos[1], estimated_probe_pos[2]))

        gcmd.respond_info("Final position: (%f,%f,%f)" % (estimated_probe_pos[0], estimated_probe_pos[1], estimated_probe_pos[2]))

def load_config(config):
    return FindProbe(config)
