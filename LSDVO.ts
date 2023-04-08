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
  trackingIsGood: boolean = true;

  keyframesAll: Frame[];
  numkeyframes: number = 0
  mapping: boolean = false;

  // PUSHED in tracking, READ & CLEARED in mapping
  unmappedTrackedFrames: Frame[] = [];
  keyFrameCtx: CanvasRenderingContext2D;
  GradCtx: CanvasRenderingContext2D;
  debug: boolean = true;
  constructor(mapping: boolean, keyFrameCtx: CanvasRenderingContext2D, GradCtx: CanvasRenderingContext2D, debug: boolean) {
    this.keyFrameCtx = keyFrameCtx;
    this.GradCtx = GradCtx;
    this.debug = debug
    this.mapping = mapping
    if (this.mapping) this.keyframesAll = [];
  }

  randomInit(image: Float32Array, width: number, height: number): void {
    this.map = new DepthMap(width, height);
    // New currentKeyframe
    this.currentKeyFrame = new Frame(image, width, height);
    this.currentKeyFrame.isKF = true;

    // Initialize map
    this.map.initializeRandomly(this.currentKeyFrame);
    if (this.debug)
      this.map.debugPlotDepthMap(this.keyFrameCtx, this.GradCtx, 1);
    console.log("Done random initialization.");
  }

  trackFrame(image: Float32Array, width: number, height: number): void {

    let trackingNewFrame: Frame = new Frame(image, width, height);
    let tracker = new SE3Tracker(width, height);
    // Set tracking reference to be the currentKeyFrame
    //this.currentKeyFrame.depthHasBeenUpdatedFlag = false;

    let frameToReference_initialEstimate: SE3 = new SE3();
    if (this.currentKeyFrame.trackedOnPoses.length > 0) {
      frameToReference_initialEstimate = this.currentKeyFrame.trackedOnPoses
      [this.currentKeyFrame.trackedOnPoses.length - 1];
    }

    console.time("track")
    let newRefToFrame_poseUpdate: SE3 = tracker.trackFrame(this.currentKeyFrame, trackingNewFrame,
      frameToReference_initialEstimate);
    console.timeEnd("track")
    console.log("lastGoodCount " + tracker.lastGoodCount);
    console.log("lastBadCount " + tracker.lastBadCount);
    if (this.debug) this.map.debugPlotDepthMap(this.keyFrameCtx, this.GradCtx, 1);
    if (Constants.manualTrackingLossIndicated || tracker.diverged
      || (this.numkeyframes > Constants.INITIALIZATION_PHASE_COUNT && !tracker.trackingWasGood)) {
      console.log("numkeyframes: " + this.numkeyframes);
      console.log("tracker.trackingWasGood: " + tracker.trackingWasGood);
      console.log("tracker.diverged: " + tracker.diverged);
      console.log("size: "
        + (trackingNewFrame.width(Constants.SE3TRACKING_MIN_LEVEL) * trackingNewFrame.height(Constants.SE3TRACKING_MIN_LEVEL)));
      this.trackingIsGood = false;
      return;
    }
    // Keyframe selection
    if (!this.createNewKeyFrame && this.currentKeyFrame.numMappedOnThisTotal > Constants.MIN_NUM_MAPPED) {

      let dist: Float32Array = Vec.scalarMult2(newRefToFrame_poseUpdate.getTranslation(), this.currentKeyFrame.meanIdepth);
      let minVal: number = Math.min(0.2 + this.numkeyframes * 0.8 / Constants.INITIALIZATION_PHASE_COUNT, 1.0);
      if (this.numkeyframes < Constants.INITIALIZATION_PHASE_COUNT)
        minVal *= 0.7;

      const lastTrackingClosenessScore = this.getRefFrameScore(Vec.dot(dist, dist), tracker.pointUsage);
      if (lastTrackingClosenessScore > minVal) {
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
      console.log("doMappingIteration: currentKeyFrame is null!");
      return;
    }
    // set mappingFrame
    if (this.trackingIsGood && this.createNewKeyFrame) {
      console.log("doMappingIteration: create new keyframe");
      // create new key frame
      this.finishCurrentKeyframe();
      this.createNewCurrentKeyframe(trackingNewFrame);
      if (this.mapping) {
        try {
          Constants.writePointCloudToFile(this.keyframesAll);
        } catch (e) {
          console.log(e)
        }
      }
      return
    } else if (!this.trackingIsGood) { // Tracking is not good
      console.log("Tracking was bad!");
      this.trackingIsGood = true
      if (this.map.isValid()) {
        if (this.currentKeyFrame.numMappedOnThisTotal >= Constants.MIN_NUM_MAPPED)
          console.log("map.invalidate");
        this.finishCurrentKeyframe();
      }
    }
    this.updateKeyframe();
  }

  // Updates key frame with measurements from a new frame.
  updateKeyframe(): void {
    // clone list
    if (this.unmappedTrackedFrames.length > 0) {
      // Copy from unmappedTrackedFrames to references
      // references - list of frames to map
      console.time("updatekeyframe")
      //	this.unmappedTrackedFrames[0]._refPixelWasGood=null
      this.map.updateKeyframe(this.unmappedTrackedFrames);
      console.timeEnd("updatekeyframe")
      this.unmappedTrackedFrames.length = 0
    } else console.log("updateKeyFrame: false");
  }

  finishCurrentKeyframe(): void {
    console.log("FINALIZING KF: " + this.currentKeyFrame.id);
    this.map.finalizeKeyFrame();
    this.numkeyframes++
    if (this.mapping) {
      this.currentKeyFrame.camToWorld = this.currentKeyFrame.getScaledCamToWorld()
      this.currentKeyFrame.clearData();
      this.keyframesAll.push(this.currentKeyFrame);
    }
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
