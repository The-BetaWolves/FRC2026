# custom lab field

## turn on
- USE_LAB_FIELD = true in VisionConstants
- upload the json to both photonvision cams (settings > field layout)
- upload the fmap to the limelight web ui
- deploy

## comp ready
- USE_LAB_FIELD = false
- put the official field back on both photon cams and the limelight
- deploy, make sure the yellow lab field alert is gone on boot

if tags move: fix the json, rebuild the fmap in limelight map builder, reupload everything
