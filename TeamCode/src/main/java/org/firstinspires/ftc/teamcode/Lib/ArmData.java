package org.firstinspires.ftc.teamcode.Lib;

public class ArmData {
    public double getArmSetAngle(ArmPos _armPos) {
        if (_armPos == ArmPos.STRAIGHT) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleStraight_22291;
            }
            return k.SHOULDER.AngleStraight_14623;
        }
        if (_armPos == ArmPos.STACK_5) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleStack_5_22291;
            }
            return k.SHOULDER.AngleStack_5_14623;
        }
        if (_armPos == ArmPos.FLOOR) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleFloor_22291;
            }
            return k.SHOULDER.AngleFloor_14623;
        }
        if (_armPos == ArmPos.STACK_3) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleStack_3_22291;
            }
            return k.SHOULDER.AngleStack_3_14623;
        }
        if (_armPos == ArmPos.BACKDROPUPLIMIT) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleBackdropUpLimit_22291;
            }
            return k.SHOULDER.AngleBackdropUpLimit_14623;
        }
        if(_armPos == ArmPos.VERTICAL){
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleVertical_22291;
            }
            return k.SHOULDER.AngleVertical_14623;
        }

        if(_armPos == ArmPos.ANGLE_BACKDROP){
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleBackdrop_22291;
            }
            return k.SHOULDER.AngleBackdrop_14623;
        }
        return k.SHOULDER.AngleStraight_14623;

    }
}
