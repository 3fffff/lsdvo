export class DepthMapPixelHypothesis {
  /**
   * Flag telling if there is a valid estimate at this point. All other values are
   * only valid if this is set to true.
   */
  public isValid: boolean;

  /**
   * Flag that blacklists a point to never be used - set if stereo fails
   * repeatedly on this pixel.
   */
  public blacklisted: number;
  /**
   * Counter for validity, basically how many successful observations are
   * incorporated.
   */
  public validity_counter: number;

  /**
   * Actual Gaussian Distribution.
   */
  public idepth: number;

  public idepth_var: number;

  /**
   * Smoothed Gaussian Distribution.
   */
  public idepth_smoothed: number;

  public idepth_var_smoothed: number;

  constructor(...args: any[]) {
    if (args.length === 1 && args[0] instanceof DepthMapPixelHypothesis) {
      const src = args[0];
      this.isValid = src.isValid;
      this.blacklisted = src.blacklisted;
      this.validity_counter = src.validity_counter;
      this.idepth = src.idepth;
      this.idepth_var = src.idepth_var;
      this.idepth_smoothed = src.idepth_smoothed;
      this.idepth_var_smoothed = src.idepth_var_smoothed;
    } else {
      this.isValid = false;
      this.blacklisted = 0;
      this.validity_counter = 0;
      this.idepth = 0;
      this.idepth_var = 0;
      this.idepth_smoothed = -1;
      this.idepth_var_smoothed = -1;
    }
  }

  getVisualizationColor(debugDisplay: number): Array<number> {
    if (debugDisplay === 0 || debugDisplay === 1) {
      let id: number = this.idepth_smoothed;
      if (id < 0) return [255, 255, 255];
      let r: number = (0.0 - id) * 255.0 / 1.0;
      if (r < 0) r = -r;
      let g: number = (1.0 - id) * 255.0 / 1.0;
      if (g < 0) g = -g;
      let b: number = (2.0 - id) * 255.0 / 1.0;
      if (b < 0) b = -b;
      let rc: number = (r < 0 ? 0 : (r > 255 ? 255 : r));
      let gc: number = (g < 0 ? 0 : (g > 255 ? 255 : g));
      let bc: number = (b < 0 ? 0 : (b > 255 ? 255 : b));
      return [(255 - rc), (255 - gc), (255 - bc)];
    }
    if (debugDisplay === 2) {
      let f: number = this.validity_counter * (255.0 / (2 * debugDisplay));
      let v: number = (f < 0 ? 0 : (f > 255 ? 255 : f));
      return [0, v, v];
    }
    if (debugDisplay === 3 || debugDisplay === 4) {
      let idv: number;
      if (debugDisplay === 3) idv = this.idepth_var_smoothed; else idv = this.idepth_var;
      let _var: number = -0.5 * (x => Math.log(x) * Math.LOG10E)(idv);
      _var = _var * 255 * 0.333;
      if (_var > 255) _var = 255;
      if (_var < 0) return [0, 0, 255];
      return [(255 - _var), _var, 0];
    }
    return [255, 255, 255];
  }

  copyFrom(other: DepthMapPixelHypothesis): void {
    this.isValid = other.isValid;
    this.blacklisted = other.blacklisted;
    this.validity_counter = other.validity_counter;
    this.idepth = other.idepth;
    this.idepth_var = other.idepth_var;
    this.idepth_smoothed = other.idepth_smoothed;
    this.idepth_var_smoothed = other.idepth_var_smoothed;
  }

  copyFromVal(idepth: number, idepth_var: number, validity_counter: number, idepth_smoothed: number = -1, idepth_var_smoothed: number = -1) {
    this.isValid = true;
    this.blacklisted = 0;
    this.validity_counter = validity_counter;
    this.idepth = idepth;
    this.idepth_var = idepth_var;
    this.idepth_smoothed = idepth_smoothed;
    this.idepth_var_smoothed = idepth_var_smoothed;
  }
}