export class DepthMapPixelHypothesis {
  /**
   * Flag telling if there is a valid estimate at this point. All other values are
   * only valid if this is set to true.
   */
  public isValid: boolean = false;

  /**
   * Flag that blacklists a point to never be used - set if stereo fails
   * repeatedly on this pixel.
   */
  public blacklisted: number = 0;

  /**
   * How many frames to skip ahead in the tracked-frames-queue.
   */
  public nextStereoFrameMinID: number = 0;

  /**
   * Counter for validity, basically how many successful observations are
   * incorporated.
   */
  public validity_counter: number = 0;

  /**
   * Actual Gaussian Distribution.
   */
  public idepth: number = 0;

  public idepth_var: number = 0;

  /**
   * Smoothed Gaussian Distribution.
   */
  public idepth_smoothed: number = 0;

  public idepth_var_smoothed: number = 0;


  public constructor(idepth?: any, idepth_smoothed?: number, idepth_var?: any, idepth_var_smoothed?: any, my_validity_counter?: any) {
    if (((typeof idepth === 'number') || idepth === null) && ((typeof idepth_smoothed === 'number') || idepth_smoothed === null) && ((typeof idepth_var === 'number') || idepth_var === null) && ((typeof idepth_var_smoothed === 'number') || idepth_var_smoothed === null) && ((typeof my_validity_counter === 'number') || my_validity_counter === null)) {
      this.isValid = true;
      this.blacklisted = 0;
      this.nextStereoFrameMinID = 0;
      this.validity_counter = my_validity_counter;
      this.idepth = idepth;
      this.idepth_var = idepth_var;
      this.idepth_smoothed = idepth_smoothed;
      this.idepth_var_smoothed = idepth_var_smoothed;
    } else if (((typeof idepth === 'number') || idepth === null) && ((typeof idepth_smoothed === 'number') || idepth_smoothed === null) && ((typeof idepth_var === 'number') || idepth_var === null) && idepth_var_smoothed === undefined && my_validity_counter === undefined) {
      let __args = arguments;
      let idepth_var: any = __args[1];
      let my_validity_counter: any = __args[2];
      this.isValid = true;
      this.blacklisted = 0;
      this.nextStereoFrameMinID = 0;
      this.validity_counter = my_validity_counter;
      this.idepth = idepth;
      this.idepth_var = idepth_var;
      this.idepth_smoothed = -1;
      this.idepth_var_smoothed = -1;
    } else if (idepth instanceof DepthMapPixelHypothesis) {
      this.isValid = idepth.isValid;
      this.blacklisted = idepth.blacklisted;
      this.nextStereoFrameMinID = idepth.nextStereoFrameMinID;
      this.validity_counter = idepth.validity_counter;
      this.idepth = idepth.idepth;
      this.idepth_var = idepth.idepth_var;
      this.idepth_smoothed = idepth.idepth_smoothed;
      this.idepth_var_smoothed = idepth.idepth_var_smoothed;
    }
  }

  getVisualizationColor(color:number): Uint8Array {
    if (color === 0 || color === 1) {
      let id: number = this.idepth_smoothed;
      if (id < 0) return new Uint8Array([255, 255, 255]);
      let r: number = (0.0 - id) * 255.0 / 1.0;
      if (r < 0) r = -r;
      let g: number = (1.0 - id) * 255.0 / 1.0;
      if (g < 0) g = -g;
      let b: number = (2.0 - id) * 255.0 / 1.0;
      if (b < 0) b = -b;
      let rc: number = (r < 0 ? 0 : (r > 255 ? 255 : r));
      let gc: number = (g < 0 ? 0 : (g > 255 ? 255 : g));
      let bc: number = (b < 0 ? 0 : (b > 255 ? 255 : b));
      return new Uint8Array([(255 - rc), (255 - gc), (255 - bc)]);
    }
    if (color === 2) {
      let f: number = this.validity_counter * (255.0 / (color + color));
      let v: number = (f < 0 ? 0 : (f > 255 ? 255 : f));
      return new Uint8Array([0, v, v]);
    }
    if (color === 3 || color === 4) {
      let idv: number;
      if (color === 3) idv = this.idepth_var_smoothed; else idv = this.idepth_var;
      let _var: number = -0.5 * (x => Math.log(x) * Math.LOG10E)(idv);
      _var = _var * 255 * 0.333;
      if (_var > 255) _var = 255;
      if (_var < 0) return new Uint8Array([0, 0, 255]);
      return new Uint8Array([(255 - _var), _var, 0]);
    }
    return new Uint8Array([255, 255, 255]);
  }
}