#pragma once

extern void drawTrail(float steeringAngle, float arcLen, float viewWidth, float ViewHeight, bool applyTransform = false);
extern void drawTrailInBEV(float steeringAngle, float arcLen);
extern void drawTrailInSingleView(float steeringAngle, float arcLen);