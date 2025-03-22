package frc.robot.RepetitiveLogic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class CoordinateTargets {
    

    public Pose2d determineCoordinates(double aprilTagID, int stepNumber, boolean intaking, boolean rightSide) {
        double xTarget = 0.0;
        double yTarget = 0.0;
        double rotationTarget = 0.0;
        
        if (intaking == false) {
            if (rightSide == true) {
                if (stepNumber == 0) {
                    if (aprilTagID == 1) {
                        xTarget = 17.55 - 1.606;
                        yTarget = 0.603;
                        rotationTarget = 126;
                    } else if (aprilTagID == 2) {
                        xTarget = 17.55 - 1.52;
                        yTarget = 7.364;
                        rotationTarget = -126;
                    } else if (aprilTagID == 12) {
                        xTarget = 1.606;
                        yTarget = 0.603;
                        rotationTarget = -126;
                    } else if (aprilTagID == 13) {
                        xTarget = 1.52;
                        yTarget = 7.364;
                        rotationTarget = 126;
                    } 
                    if (aprilTagID == 6) {
                      xTarget = Constants.kRedBottomRR[0];
                      yTarget = Constants.kRedBottomRR[1];
                      rotationTarget = Constants.kRedBottomRR[2];
                    } else if (aprilTagID == 7) {
                      xTarget = Constants.kRedRightR[0];
                      yTarget = Constants.kRedRightR[1];
                      rotationTarget = Constants.kRedRightR[2];
                    } else if (aprilTagID == 8) {
                      xTarget = Constants.kRedTopRR[0];
                      yTarget = Constants.kRedTopRR[1];
                      rotationTarget = Constants.kRedTopRR[2];
                    } else if (aprilTagID == 9) {
                      xTarget = Constants.kRedTopLR[0];
                      yTarget = Constants.kRedTopLR[1];
                      rotationTarget = Constants.kRedTopLR[2];
                    } else if (aprilTagID == 10) {
                      xTarget = Constants.kRedLeftR[0];
                      yTarget = Constants.kRedLeftR[1];
                      rotationTarget = Constants.kRedLeftR[2];
                    } else if (aprilTagID == 11) {
                      xTarget = Constants.kRedBottomLR[0];
                      yTarget = Constants.kRedBottomLR[1];
                      rotationTarget = Constants.kRedBottomLR[2];
                    } else if (aprilTagID == 17) {
                      xTarget = (Constants.kBlueBottomLR[0] - 0.5);
                      yTarget = (Constants.kBlueBottomLR[1] - 0.86603);
                      rotationTarget = Constants.kBlueBottomLR[2];
                    } else if (aprilTagID == 18) {
                      xTarget = (Constants.kBlueLeftR[0] - 1);
                      yTarget = Constants.kBlueLeftR[1];
                      rotationTarget = Constants.kBlueLeftR[2];
                    } else if (aprilTagID == 19) {
                      xTarget = (Constants.kBlueTopLR[0] - 0.5);
                      yTarget = (Constants.kBlueTopLR[1] + 0.86603);
                      rotationTarget = Constants.kBlueTopLR[2];
                    } else if (aprilTagID == 20) {
                      xTarget = (Constants.kBlueTopRR[0] + 0.5);
                      yTarget = (Constants.kBlueTopRR[1] + 0.86603);
                      rotationTarget = Constants.kBlueTopRR[2];
                    } else if (aprilTagID == 21) {
                      xTarget = (Constants.kBlueRightR[0] + 1);
                      yTarget = Constants.kBlueRightR[1];
                      rotationTarget = Constants.kBlueRightR[2];
                    } else if (aprilTagID == 22) {
                      xTarget = (Constants.kBlueBottomRR[0] + 0.5);
                      yTarget = (Constants.kBlueBottomRR[1] - 0.86603);
                      rotationTarget = Constants.kBlueBottomRR[2];
                    }
                } 
                if (stepNumber == 1) {
                  if (aprilTagID == 6) {
                    xTarget = Constants.kRedBottomRR[0];
                    yTarget = Constants.kRedBottomRR[1];
                    rotationTarget = Constants.kRedBottomRR[2];
                  } else if (aprilTagID == 7) {
                    xTarget = Constants.kRedRightR[0];
                    yTarget = Constants.kRedRightR[1];
                    rotationTarget = Constants.kRedRightR[2];
                  } else if (aprilTagID == 8) {
                    xTarget = Constants.kRedTopRR[0];
                    yTarget = Constants.kRedTopRR[1];
                    rotationTarget = Constants.kRedTopRR[2];
                  } else if (aprilTagID == 9) {
                    xTarget = Constants.kRedTopLR[0];
                    yTarget = Constants.kRedTopLR[1];
                    rotationTarget = Constants.kRedTopLR[2];
                  } else if (aprilTagID == 10) {
                    xTarget = Constants.kRedLeftR[0];
                    yTarget = Constants.kRedLeftR[1];
                    rotationTarget = Constants.kRedLeftR[2];
                  } else if (aprilTagID == 11) {
                    xTarget = Constants.kRedBottomLR[0];
                    yTarget = Constants.kRedBottomLR[1];
                    rotationTarget = Constants.kRedBottomLR[2];
                  } else if (aprilTagID == 17) {
                    xTarget = Constants.kBlueBottomLR[0];
                    yTarget = Constants.kBlueBottomLR[1];
                    rotationTarget = Constants.kBlueBottomLR[2];
                  } else if (aprilTagID == 18) {
                    xTarget = Constants.kBlueLeftR[0];
                    yTarget = Constants.kBlueLeftR[1];
                    rotationTarget = Constants.kBlueLeftR[2];
                  } else if (aprilTagID == 19) {
                    xTarget = Constants.kBlueTopLR[0];
                    yTarget = Constants.kBlueTopLR[1];
                    rotationTarget = Constants.kBlueTopLR[2];
                  } else if (aprilTagID == 20) {
                    xTarget = Constants.kBlueTopRR[0];
                    yTarget = Constants.kBlueTopRR[1];
                    rotationTarget = Constants.kBlueTopRR[2];
                  } else if (aprilTagID == 21) {
                    xTarget = Constants.kBlueRightR[0];
                    yTarget = Constants.kBlueRightR[1];
                    rotationTarget = Constants.kBlueRightR[2];
                  } else if (aprilTagID == 22) {
                    xTarget = Constants.kBlueBottomRR[0];
                    yTarget = Constants.kBlueBottomRR[1];
                    rotationTarget = Constants.kBlueBottomRR[2];
                  }
                }
            } else if (rightSide == false) {
                if (stepNumber == 0) {
                    if (aprilTagID == 1) {
                        xTarget = 17.55 - 1.606;
                        yTarget = 0.603;
                        rotationTarget = 126;
                    } else if (aprilTagID == 2) {
                        xTarget = 17.55 - 1.52;
                        yTarget = 7.364;
                        rotationTarget = -126;
                    } else if (aprilTagID == 12) {
                        xTarget = 1.606;
                        yTarget = 0.603;
                        rotationTarget = -126;
                    } else if (aprilTagID == 13) {
                        xTarget = 1.52;
                        yTarget = 7.364;
                        rotationTarget = 126;
                    } 
                    if (aprilTagID == 6) {
                      xTarget = Constants.kRedBottomRL[0];
                      yTarget = Constants.kRedBottomRL[1];
                      rotationTarget = Constants.kRedBottomRL[2];
                    } else if (aprilTagID == 7) {
                      xTarget = Constants.kRedRightL[0];
                      yTarget = Constants.kRedRightL[1];
                      rotationTarget = Constants.kRedRightL[2];
                    } else if (aprilTagID == 8) {
                      xTarget = Constants.kRedTopRL[0];
                      yTarget = Constants.kRedTopRL[1];
                      rotationTarget = Constants.kRedTopRL[2];
                    } else if (aprilTagID == 9) {
                      xTarget = Constants.kRedTopLL[0];
                      yTarget = Constants.kRedTopLL[1];
                      rotationTarget = Constants.kRedTopLL[2];
                    } else if (aprilTagID == 10) {
                      xTarget = Constants.kRedLeftL[0];
                      yTarget = Constants.kRedLeftL[1];
                      rotationTarget = Constants.kRedLeftL[2];
                    } else if (aprilTagID == 11) {
                      xTarget = Constants.kRedBottomLL[0];
                      yTarget = Constants.kRedBottomLL[1];
                      rotationTarget = Constants.kRedBottomLL[2];
                    } else if (aprilTagID == 17) {
                      xTarget = (Constants.kBlueBottomLL[0] - 0.5);
                      yTarget = (Constants.kBlueBottomLL[1] - 0.86603);
                      rotationTarget = Constants.kBlueBottomLL[2];
                    } else if (aprilTagID == 18) {
                      xTarget = (Constants.kBlueLeftL[0] - 1);
                      yTarget = Constants.kBlueLeftL[1];
                      rotationTarget = Constants.kBlueLeftL[2];
                    } else if (aprilTagID == 19) {
                      xTarget = (Constants.kBlueTopLL[0] - 0.5);
                      yTarget = (Constants.kBlueTopLL[1] + 0.86603);
                      rotationTarget = Constants.kBlueTopLL[2];
                    } else if (aprilTagID == 20) {
                      xTarget = (Constants.kBlueTopRL[0] + 0.5);
                      yTarget = (Constants.kBlueTopRL[1] + 0.86603);
                      rotationTarget = Constants.kBlueTopRL[2];
                    } else if (aprilTagID == 21) {
                      xTarget = (Constants.kBlueRightL[0] + 1);
                      yTarget = Constants.kBlueRightL[1];
                      rotationTarget = Constants.kBlueRightL[2];
                    } else if (aprilTagID == 22) {
                      xTarget = (Constants.kBlueBottomRL[0] + 0.5);
                      yTarget = (Constants.kBlueBottomRL[1] - 0.86603);
                      rotationTarget = Constants.kBlueBottomRL[2];
                    }
                }
                if (stepNumber == 1) {
                  if (aprilTagID == 6) {
                      xTarget = Constants.kRedBottomRL[0];
                      yTarget = Constants.kRedBottomRL[1];
                      rotationTarget = Constants.kRedBottomRL[2];
                    } else if (aprilTagID == 7) {
                      xTarget = Constants.kRedRightL[0];
                      yTarget = Constants.kRedRightL[1];
                      rotationTarget = Constants.kRedRightL[2];
                    } else if (aprilTagID == 8) {
                      xTarget = Constants.kRedTopRL[0];
                      yTarget = Constants.kRedTopRL[1];
                      rotationTarget = Constants.kRedTopRL[2];
                    } else if (aprilTagID == 9) {
                      xTarget = Constants.kRedTopLL[0];
                      yTarget = Constants.kRedTopLL[1];
                      rotationTarget = Constants.kRedTopLL[2];
                    } else if (aprilTagID == 10) {
                      xTarget = Constants.kRedLeftL[0];
                      yTarget = Constants.kRedLeftL[1];
                      rotationTarget = Constants.kRedLeftL[2];
                    } else if (aprilTagID == 11) {
                      xTarget = Constants.kRedBottomLL[0];
                      yTarget = Constants.kRedBottomLL[1];
                      rotationTarget = Constants.kRedBottomLL[2];
                    } else if (aprilTagID == 17) {
                      xTarget = Constants.kBlueBottomLL[0];
                      yTarget = Constants.kBlueBottomLL[1];
                      rotationTarget = Constants.kBlueBottomLL[2];
                    } else if (aprilTagID == 18) {
                      xTarget = Constants.kBlueLeftL[0];
                      yTarget = Constants.kBlueLeftL[1];
                      rotationTarget = Constants.kBlueLeftL[2];
                    } else if (aprilTagID == 19) {
                      xTarget = Constants.kBlueTopLL[0];
                      yTarget = Constants.kBlueTopLL[1];
                      rotationTarget = Constants.kBlueTopLL[2];
                    } else if (aprilTagID == 20) {
                      xTarget = Constants.kBlueTopRL[0];
                      yTarget = Constants.kBlueTopRL[1];
                      rotationTarget = Constants.kBlueTopRL[2];
                    } else if (aprilTagID == 21) {
                      xTarget = Constants.kBlueRightL[0];
                      yTarget = Constants.kBlueRightL[1];
                      rotationTarget = Constants.kBlueRightL[2];
                    } else if (aprilTagID == 22) {
                      xTarget = Constants.kBlueBottomRL[0];
                      yTarget = Constants.kBlueBottomRL[1];
                      rotationTarget = Constants.kBlueBottomRL[2];
                    }
                } 
            }  
        
        } else {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Blue) {
                if (rightSide == true) {
                    xTarget = 1.606;
                    yTarget = 0.603;
                    rotationTarget = -126;
                } else {
                    xTarget = 1.52;
                    yTarget = 7.364;
                    rotationTarget = 126;
                }
            } else {
                if (rightSide != true) {
                    xTarget = 17.55 - 1.606;
                    yTarget = 0.603;
                    rotationTarget = 126;
                 } else {
                    xTarget = 17.55 - 1.52;
                    yTarget = 7.364;
                    rotationTarget = -126;
                 }
            }
        }

        return new Pose2d(xTarget, yTarget, new Rotation2d(rotationTarget));
    }
}
