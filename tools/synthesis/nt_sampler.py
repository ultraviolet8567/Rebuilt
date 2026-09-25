import sys, time, json, struct, ntcore
KEYS = {
 "speeds": "/AdvantageKit/RealOutputs/Drive/ChassisSpeeds",
 "setpointSpeeds": "/AdvantageKit/RealOutputs/Drive/SetpointSpeeds",
 "setpointStates": "/AdvantageKit/RealOutputs/Drive/SetpointStates",
 "pose": "/AdvantageKit/RealOutputs/Drive/Pose",
 "angleToHub": "/AdvantageKit/RealOutputs/Drive/AngleToHub",
 "wheelsLocked": "/AdvantageKit/RealOutputs/RobotState/WheelsLocked",
 "funnelV": "/AdvantageKit/Intake/Funnel/AppliedVolts",
 "indexerV": "/AdvantageKit/Storage/Indexer/AppliedVolts",
 "hoodTarget": "/AdvantageKit/RealOutputs/Shooter/Hood/TargetAngleRad",
 "hoodAngle": "/AdvantageKit/RealOutputs/Shooter/Hood/AngleRad",
 "flyRunning": "/AdvantageKit/RealOutputs/Shooter/Flywheel/Running",
 "flyTarget": "/AdvantageKit/RealOutputs/Shooter/Flywheel/TargetRpm",
 "flyRpm": "/AdvantageKit/Shooter/Flywheel/LeadVelocityRpm",
 "atSpeed": "/AdvantageKit/RealOutputs/Shooter/Flywheel/AtSpeed",
 "kickerRunning": "/AdvantageKit/RealOutputs/Shooter/Kicker/Running",
 "kickerV": "/AdvantageKit/Shooter/Kicker/AppliedVolts",
 "pivotTarget": "/AdvantageKit/RealOutputs/Intake/Pivot/TargetAngleRad",
 "pivotAngle": "/AdvantageKit/RealOutputs/Intake/Pivot/AngleRad",
 "mode": "/AdvantageKit/RealOutputs/RobotState/Mode",
 "rumble": "/AdvantageKit/DriverStation/Joystick0/ButtonValues",
}
inst = ntcore.NetworkTableInstance.create(); inst.startClient4("sampler"); inst.setServer("127.0.0.1", 5810)
sub = ntcore.MultiSubscriber(inst, ["/AdvantageKit"]); time.sleep(2)
def dec(v):
    if not v.isValid(): return None
    x = v.value()
    if isinstance(x, (bytes, bytearray)):
        n = len(x)//8; return [round(d,3) for d in struct.unpack("<%dd"%n, x[:n*8])]
    return round(x,3) if isinstance(x,float) else x
end = time.time() + float(sys.argv[1])
with open(sys.argv[2], "w") as f:
    while time.time() < end:
        row = {"t": int(time.time()*1000)}
        for k, topic in KEYS.items(): row[k] = dec(inst.getEntry(topic).getValue())
        f.write(json.dumps(row)+"\n"); f.flush(); time.sleep(0.1)
