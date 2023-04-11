import { Frame } from "./DataStructures/Frame";
import { SE3Tracker } from "./Tracking/SE3Tracker";
import { DepthMap } from "./DepthEstimation/DepthMap";
import { Constants } from "./Utils/Constants";
import { SE3 } from "./LieAlgebra/SE3";
import { Vec } from "./LieAlgebra/Vec";

export class LSDVO {
  currentKeyFrame: Frame;
  map: DepthMap;
  createNewKeyFrame: boolean = false;
  tracker: SE3Tracker
  numkeyframes: number = 0
  mapping: boolean = false;

  // PUSHED in tracking, READ & CLEARED in mapping
  unmappedTrackedFrames: Frame[] = [];
  debug: boolean = true;
  constructor(mapping: boolean, debug: boolean) {
    this.debug = debug
    this.mapping = mapping
  }

  randomInit(image: Float32Array, width: number, height: number): void {
    this.map = new DepthMap(width, height);
    this.tracker = new SE3Tracker(width, height);
    // New currentKeyframe
    this.currentKeyFrame = new Frame(image, width, height);
    this.currentKeyFrame.isKF = true;

    // Initialize map
    this.map.initializeRandomly(this.currentKeyFrame);
    if (this.debug) this.map.debugPlotDepthMap();
    console.log("Done random initialization.");
  }

  trackFrame(image: Float32Array, width: number, height: number): void {
    let trackingNewFrame: Frame = new Frame(image, width, height);

    let frameToReference_initialEstimate: SE3 = this.currentKeyFrame.trackedOnPoses.length > 0 ? this.currentKeyFrame.trackedOnPoses
    [this.currentKeyFrame.trackedOnPoses.length - 1] : new SE3();

    console.time("track")
    let newRefToFrame_poseUpdate: SE3 = this.tracker.trackFrame(this.currentKeyFrame, trackingNewFrame,
      frameToReference_initialEstimate);
    console.timeEnd("track")
    console.log("lastGoodCount " + this.tracker.lastGoodCount);
    console.log("lastBadCount " + this.tracker.lastBadCount);
    if (this.debug) this.map.debugPlotDepthMap();
    if (this.tracker.diverged || (this.numkeyframes > Constants.INITIALIZATION_PHASE_COUNT && !this.tracker.trackingWasGood)) {
      console.log("tracker.diverged: " + this.tracker.diverged);
      return;
    }
    // Keyframe selection
    if (!this.createNewKeyFrame && this.currentKeyFrame.numMappedOnThisTotal > Constants.MIN_NUM_MAPPED) {

      let dist: Float32Array = Vec.scalarMult2(newRefToFrame_poseUpdate.getTranslation(), this.currentKeyFrame.meanIdepth);
      let minVal: number = Math.min(0.2 + this.numkeyframes * 0.8 / Constants.INITIALIZATION_PHASE_COUNT, 1.0);
      if (this.numkeyframes < Constants.INITIALIZATION_PHASE_COUNT)
        minVal *= 0.7;

      if (this.getRefFrameScore(Vec.dot(dist, dist), this.tracker.pointUsage) > minVal) {
        console.log("CREATE NEW KEYFRAME");
        this.createNewKeyFrame = true;
        trackingNewFrame.isKF = true;
      }
    }
    // Push into deque for mapping
    this.unmappedTrackedFrames.push(trackingNewFrame);
    this.doMappingIteration(trackingNewFrame)
  }

  doMappingIteration(trackingNewFrame: Frame): void | null {
    if (this.currentKeyFrame == null) {
      console.error("doMappingIteration: currentKeyFrame is null!");
      return;
    }
    this.updateKeyframe();
    // set mappingFrame
    if (this.tracker.trackingWasGood&& this.createNewKeyFrame) {
      console.log("doMappingIteration: create new keyframe");
      if (this.mapping) {
        try {
          Constants.writePointCloudToFile(this.currentKeyFrame);
        } catch (e) {
          console.log(e)
        }
      }
      // create new key frame
      this.finishCurrentKeyframe();
      this.createNewCurrentKeyframe(trackingNewFrame);
    } else if (!this.tracker.trackingWasGood) { // Tracking is not good
      console.error("Tracking was bad!");
      if (this.map.isValid()) {
        if (this.currentKeyFrame.numMappedOnThisTotal >= Constants.MIN_NUM_MAPPED)
          console.log("map.invalidate");
        this.finishCurrentKeyframe();
      }
    }
  }

  // Updates key frame with measurements from a new frame.
  updateKeyframe(): void {
    // clone list
    if (this.unmappedTrackedFrames.length > 0) {
      // Copy from unmappedTrackedFrames to references
      // references - list of frames to map
      console.time("updatekeyframe")
      this.map.updateKeyframe(this.unmappedTrackedFrames);
      console.timeEnd("updatekeyframe")
      this.unmappedTrackedFrames.length = 0
    } else console.log("updateKeyFrame: false");
  }

  finishCurrentKeyframe(): void {
    console.log("FINALIZING KF: " + this.currentKeyFrame.id);
    this.map.finalizeKeyFrame();
    this.numkeyframes++
  }

  createNewCurrentKeyframe(newKeyframeCandidate: Frame): void {
    console.log("CREATE NEW KF %d from %d\n", newKeyframeCandidate.id, this.currentKeyFrame.id);
    this.createNewKeyFrame = false;
    this.map.createKeyFrame(newKeyframeCandidate);
    this.currentKeyFrame = newKeyframeCandidate;
  }

  getRefFrameScore(distanceSquared: number, usage: number): number {
    return distanceSquared * 12 + (1 - usage) * (1 - usage) * 12;
  }
}
