import { Constants } from "./Utils/Constants";
import { Frame } from "./DataStructures/Frame";
import { DepthMap } from "./DepthEstimation/DepthMap";
import { SE3 } from "./LieAlgebra/SE3";
import { SIM3 } from "./LieAlgebra/SIM3";
import { SE3Tracker } from "./Tracking/SE3Tracker";
import { Vec } from "./LieAlgebra/Vec";

export class LSDVO {

  currentKeyFrame: Frame;
  map: DepthMap;
  tracker: SE3Tracker;
  createNewKeyFrame = false;
  totalFrames: number = 0;
  mapping = true;
  debug = true;
  keyframesAll: Frame[] = [];
  allFramePoses: SIM3[] = []
  lastTrackedPose: SIM3 = new SIM3();
  // PUSHED in tracking, READ & CLEARED in mapping
  unmappedTrackedFrames: Frame[] = [];

  constructor(mapping: boolean, debug: boolean) {
    this.mapping = mapping
    this.debug = debug
  }

  randomInit(image: Float32Array, width: number, height: number) {
    this.map = new DepthMap(width, height);
    this.tracker = new SE3Tracker(width, height);

    // New currentKeyframe
    this.currentKeyFrame = new Frame(image, width, height, this.totalFrames);
    this.currentKeyFrame.isKF = true;

    // Initialize map
    this.map.initializeRandomly(this.currentKeyFrame);
    this.totalFrames++;
    if (this.debug) this.map.debugPlotDepthMap();
    console.log("Done random initialization.");
  }

  trackFrame(image: Float32Array, width: number, height: number) {

    let trackingNewFrame: Frame = new Frame(image, width, height, this.totalFrames);

    let frameToReference_initialEstimate: SE3 = this.getFrameInitialEstimate();
    console.log("frameToReference_initialEstimate: " + SE3.ln(frameToReference_initialEstimate));

    let newRefToFrame_poseUpdate: SE3 = this.tracker.trackFrame(this.currentKeyFrame, trackingNewFrame,
      frameToReference_initialEstimate);

    this.lastTrackedPose = trackingNewFrame.thisToParent;
    if (this.tracker.diverged || (this.totalFrames > Constants.INITIALIZATION_PHASE_COUNT && !this.tracker.trackingWasGood)) {
      console.log("keyframesAll.size(): " + this.totalFrames);
      console.log("tracker.trackingWasGood: " + this.tracker.trackingWasGood);
      console.log("tracker.diverged: " + this.tracker.diverged);
      return;
    }

    this.totalFrames++;
    this.createNewKeyFrame = false;
    // Keyframe selection

    if (this.currentKeyFrame.numMappedOnThisTotal > Constants.MIN_NUM_MAPPED) {

      let dist: Float32Array = Vec.scalarMult2(newRefToFrame_poseUpdate.getTranslation(), this.currentKeyFrame.meanIdepth);
      let minVal: number = Math.min(0.2 + this.totalFrames * 0.8 / Constants.INITIALIZATION_PHASE_COUNT, 1.0);

      if (this.totalFrames < Constants.INITIALIZATION_PHASE_COUNT)
        minVal *= 0.7;

      let lastTrackingClosenessScore: number = this.getRefFrameScore(Vec.dot(dist, dist), this.tracker.pointUsage);

      if (lastTrackingClosenessScore > minVal) {
        console.log("CREATE NEW KEYFRAME");
        this.createNewKeyFrame = true;
        trackingNewFrame.isKF = true;
      }
    }
    // Push into deque for mapping
    this.unmappedTrackedFrames.push(trackingNewFrame);
    this.doMappingIteration(trackingNewFrame);
  }

  doMappingIteration(trackingNewFrame: Frame) {
    if (this.currentKeyFrame == null)
      console.log("doMappingIteration: currentKeyFrame is null!");

    // set mappingFrame
    if (this.tracker.trackingWasGood) {
      console.log("Tracking was good!");
      if (this.createNewKeyFrame) {
        this.map.finalizeKeyFrame();
        console.log("CREATE NEW KF %d from %d\n", trackingNewFrame.id, this.currentKeyFrame.id);
        // propagate & make new.
        this.map.createKeyFrame(trackingNewFrame);
        this.currentKeyFrame = trackingNewFrame;
        this.unmappedTrackedFrames.length = 0;
        this.allFramePoses.push(trackingNewFrame.thisToParent);
        if (this.mapping) {
          this.keyframesAll.push(this.currentKeyFrame);
          Constants.writePointCloudToFile(this.keyframesAll);
        }
      } else {
        console.log("doMappingIteration: update keyframe");
        // ***Update key frame here***
        if (this.unmappedTrackedFrames.length > 0)
          this.map.updateKeyframe(this.unmappedTrackedFrames);
      }
    } else { // Tracking is not good
      console.log("Tracking was bad!");

      // invalidate map if it was valid.
      if (this.map.isValid()) {
        if (this.currentKeyFrame.numMappedOnThisTotal >= Constants.MIN_NUM_MAPPED) {
          console.log("FINALIZING KF: " + this.currentKeyFrame.id);
          this.map.finalizeKeyFrame();
        }
        console.log("map.invalidate");
      }
    }
    if (this.debug) this.map.debugPlotDepthMap();
  }

  getRefFrameScore(distanceSquared: number, usage: number): number {
    return distanceSquared * 3 * 4 + (1 - usage) * (1 - usage) * 3 * 4;
  }

  getFrameInitialEstimate(): SE3 {
    let camToWorld: SIM3 = new SIM3();
    let camToWorldR: SIM3 = new SIM3();

    let lastCount: number = this.createNewKeyFrame ? 1 : 0;
    for (let i: number = 0; i < this.allFramePoses.length - lastCount; i++)
      camToWorldR = camToWorldR.mul(this.allFramePoses[i]);
    for (let i: number = 0; i < this.allFramePoses.length; i++)
      camToWorld = camToWorld.mul(this.allFramePoses[i]);
    return camToWorld.getSE3().inverse().mul(camToWorldR.mul(this.lastTrackedPose).getSE3());
  }
}
