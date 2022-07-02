#!/usr/bin/env pybricks-micropython

#! BELANGRIJKE INFORMATIE
# De suffix "stab_" (in ieder mogelijke combinatie van hoofd en kleine letters) duidt op de stabilisatie of de periode waarin
# de stabilisatie gebeurt tenzij anders aangegeven; dit maakt duidelijk dat deze "dingen" alleen een functie hebben
# gedurende de stabilisatie periode - dit betekend tevens niet dat "dingen" zonder de prefix "stab" (in iedere mogelijke
# combinatie van hoofd en kleine letters) niet worden gebruikt voor de stabilisatie noch niet exclusief in de voor 
# stabilisatie gebruikt worden jegens code leesbaarheid.

from ucollections import namedtuple
import urandom
import math

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.nxtdevices import UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait, StopWatch

# Global definitions (constants); waardes komen van pybricks documentatie code
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  

#? Wordt gebruikt om te bepalen welke richting de robot rijdt.
Stab_action = namedtuple('Action ', ['drive_speed', 'steering'])
STOP = Stab_action(drive_speed=0, steering=0)
FORWARD_FAST = Stab_action(drive_speed=100, steering=0)
FORWARD_SLOW = Stab_action(drive_speed=40, steering=0)
BACKWARD_FAST = Stab_action(drive_speed=-65, steering=0)
BACKWARD_SLOW = Stab_action(drive_speed=-10, steering=0)
TURN_RIGHT = Stab_action(drive_speed=0, steering=70)
TURN_LEFT = Stab_action(drive_speed=0, steering=-70)

#? Alleen gedurende stabilisatie jegens verschil in motor opbouw/gebruik vergeleke met wanneer de robot op 4 wielen staat
STAB_ACTION_MAP = {
    Color.RED: STOP,
    Color.GREEN: FORWARD_FAST,
    Color.BLUE: TURN_RIGHT,
    Color.YELLOW: TURN_LEFT,
    Color.WHITE: BACKWARD_FAST,
}

class DriveBase:
    def __init__(self, motor_rl: Motor, motor_rr: Motor, motor_fl: Motor, motor_fr: Motor):
        self.rear_left = motor_rl
        self.rear_right = motor_rr
        self.front_left = motor_fl
        self.front_right = motor_fr

    # <param name="speed">rotational speed (deg/s)</param>
    def drive(self, speed: float):
        self.front_left.run(speed)
        self.front_right.run(speed)
        self.rear_left.run(speed * -1)
        self.rear_right.run(speed * -1)

    # <param name="speed">arbitraire snelheids value</param>
    # <param name="duration">(ms)</param>
    def turn(self, speed: float, duration: int):
        self.stop_all()
        self.front_left.run(speed)
        self.front_right.run(0.2 * speed)
        self.rear_left.run(speed * -1)
        self.rear_right.run(0.2 * speed * -1)
        wait(duration)
        self.stop_all()

    def stop_all(self):
        self.front_left.stop()
        self.front_right.stop()
        self.rear_left.stop()
        self.rear_right.stop()

    def reset_angle_all(self):
        self.front_left.reset_angle(0)
        self.front_right.reset_angle(0)
        self.rear_left.reset_angle(0)
        self.rear_right.reset_angle(0)

class Robot:
    #? Fields worden hier (bijna) niet beschreven (aan de hand van commentaar): dit wordt (waar nodig) gedaan waar ze hun functie dienen
    def __init__(self, brick: EV3Brick,
        motor_rl: Motor, motor_rr: Motor, motor_fl: Motor, motor_fr: Motor,
        color_sensor: ColorSensor, gyro_sensor: GyroSensor, ultrasonic_sensor: UltrasonicSensor
        ):

        self.brick = brick

        self.drive_base = DriveBase(
            motor_rr=motor_rr,
            motor_rl=motor_rl,
            motor_fr=motor_fr,
            motor_fl=motor_fl,
        )
        self.stab_wheel_angle = 0

        self.sensor_color = color_sensor
        self.sensor_ultrasonic = ultrasonic_sensor
        self.sensor_gyro = gyro_sensor
        self.gyro_offset = 0 # wordt in calibrate_gyrosensor() berekend

        self.timers = {
            "loop": StopWatch(),
            "control_loop": StopWatch(),
            "stab_fall": StopWatch(),
            "stab_action": StopWatch(),
        }
        self.counts = {
            "loop": 0,
            "control_loop": 0,
        }
        self.body = {
            "angle": -0.25,
            "rate": 0,
        }

        self.stab_motor_position_change = [0, 0, 0, 0]
        self.stab_drive_speed = 0 # Alleen de drive speed gedurende de stabilisatie -> de 4 wielige aandrijving heeft dit niet nodig
        self.stab_steering = 0 # Alleen de steering gedurende de stabilisatie -> de 4 wielige aandrijving heeft dit niet nodig

    # <summary>
    #   Calibreerd de gyro sensor.
    #TODO beter commentaar
    # </summary>
    def calibrate_gyrosensor(self):
        while True:
            gyro_minimum_rate, gyro_maximum_rate = 440, -440
            gyro_sum = 0
            for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
                gyro_sensor_value = self.sensor_gyro.speed()
                gyro_sum += gyro_sensor_value
                if gyro_sensor_value > gyro_maximum_rate:
                    gyro_maximum_rate = gyro_sensor_value
                if gyro_sensor_value < gyro_minimum_rate:
                    gyro_minimum_rate = gyro_sensor_value
                wait(5)
            if gyro_maximum_rate - gyro_minimum_rate < 2:
                break
        self.gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    def stabilise_update_action(self):
        self.timers["stab_action"].reset()

        yield FORWARD_SLOW
        while self.timers["stab_action"].time() < 4000:
            yield

        action = STOP
        yield action

        while True:
            new_action = STAB_ACTION_MAP.get(self.sensor_color.color())

            if new_action is not None:
                self.timers["stab_action"].reset()
                self.brick.speaker.beep(1000, -1)
                while self.timers["stab_action"].time() < 100:
                    yield
                self.brick.speaker.beep(0, -1)

                if new_action.steering != 0:
                    action = Stab_action(drive_speed=action.drive_speed, steering=new_action.steering)
                else:
                    action = new_action
                yield action

            self.timers["stab_action"].reset()
            while self.timers["stab_action"].time() < 100:
                yield

    # <summary>
    #   De main stabilisatie functie.
    #   Doet de volgende dingen (op volgorde):
    #   Reset alle waardes (zorgt ervoor dat er geen per ongeluke contaminatie plaats kan vinden);
    #   Berekent de gemiddelde duratie vaan een loop;
    #   Berekent de in hoeverre de robot instabiel is;    
    #   Berekent in hoeverre ingegrepen moet worden;
    #   Voert vervolgens de ingreep uit.
    # </summary>
    def stabilise(self) -> bool:
        gyro_sensor_value = self.sensor_gyro.speed()
        robot_body_rate = gyro_sensor_value - self.gyro_offset
        motor_position_sum = 0
        self.drive_base.stop_all()
        self.drive_base.reset_angle_all()
        self.timers["stab_fall"].reset()
        action_task = self.stabilise_update_action()

        while True:
            self.timers["loop"].reset()

            #? Voor een of andere vage rede werkt dit niet in de main functie maar wel in deze loop???
            if self.sensor_ultrasonic.distance() < 100:
                self.drive_base.rear_right.dc(100)
                self.drive_base.rear_left.dc(100)
                wait(100)
                self.drive_base.stop_all()
                return False

            if self.counts["control_loop"] == 0:
                average_control_loop_period = TARGET_LOOP_PERIOD / 1000
                self.timers["control_loop"].reset()
            else:
                average_control_loop_period = self.timers["control_loop"].time() / 1000 / self.counts["control_loop"]
            self.counts["control_loop"] += 1

            # Berekent in hoeverre de robot instabiel is.
            # Zodat berekent kan worden in hoeverre ingegrepen moet worden.
            gyro_sensor_value = self.sensor_gyro.speed()
            self.gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
            self.gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
            self.body["rate"] = gyro_sensor_value - self.gyro_offset
            self.body["angle"] += self.body["rate"] * average_control_loop_period

            # Berekent de benodigde ingreep aan de hand van de boven berekende instabiliteit (self.body_angle)
            previous_motor_sum = motor_position_sum
            motor_position_sum = self.drive_base.rear_right.angle() + self.drive_base.rear_left.angle()
            difference = motor_position_sum - previous_motor_sum
            self.stab_motor_position_change.insert(0, difference)
            del self.stab_motor_position_change[-1]
            self.stab_wheel_angle += difference - self.stab_drive_speed * average_control_loop_period
            wheel_rate = sum(self.stab_motor_position_change) / 4 / average_control_loop_period

            output_power = (-0.01 * self.stab_drive_speed) + (0.8 * self.body["rate"] + 15 * self.body["angle"] + 0.08 * wheel_rate + 0.12 * self.stab_wheel_angle)
            #? De power is in duty (in procent dus): wanneer dit meer dan |100| (absoluut) is moet dit dus verlaagt worden naar 100
            if output_power > 100:
                output_power = 100
            if output_power < -100:
                output_power = -100

            self.drive_base.rear_right.dc(output_power - 0.1 * self.stab_steering)
            self.drive_base.rear_left.dc(output_power + 0.1 * self.stab_steering)

            # Checkt of de robot is omgevallen
            if abs(output_power) < 100:
                self.timers["stab_fall"].reset()
            elif self.timers["stab_fall"].time() > 10000:
                return False

            action = next(action_task)
            if action is not None:
                self.stab_drive_speed, self.stab_steering = action

            wait(TARGET_LOOP_PERIOD - self.timers["loop"].time()) 

def main():
    menneke = Robot(
        brick=EV3Brick(),
        motor_rl=Motor(Port.A),
        motor_rr=Motor(Port.D),
        motor_fl=Motor(Port.B),
        motor_fr=Motor(Port.C),
        gyro_sensor=GyroSensor(Port.S1),
        color_sensor=ColorSensor(Port.S3),
        ultrasonic_sensor=UltrasonicSensor(Port.S4)
    )
    
    menneke.brick.light.off()
    wait(1000)

    menneke.brick.light.on(Color.YELLOW)
    menneke.calibrate_gyrosensor()

    menneke.brick.light.on(Color.RED)

    while True:
        if not menneke.stabilise():
            break

    menneke.brick.light.on(Color.GREEN)
    menneke.drive_base.drive(720)
    wait(1000)

main()