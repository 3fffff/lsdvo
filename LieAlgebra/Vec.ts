export class Vec {
  /**
   * Calculates magnitude of a vector.
   */
  public static magnitude(vec: Array<number>): number {
    let magnitude: number = 0;
    for (let i: number = 0; i < vec.length; i++) magnitude += vec[i] * vec[i];
    return Math.sqrt(magnitude);
  }

  public static invert(a: Array<number>, len: number): Array<number> {
    let n: number = len;
    let x = Array(n * n).fill(0);
    let b = Array(n * n).fill(0);
    let index = Array(n).fill(0);
    for (let i: number = 0; i < n; ++i) b[i * n + i] = 1;
    Vec.gaussian(a, index);
    for (let i: number = 0; i < n - 1; ++i)
      for (let j: number = i + 1; j < n; ++j)
        for (let k: number = 0; k < n; ++k)
          b[index[j] * n + k] -= a[index[j] * n + i] * b[index[i] * n + k];
    for (let i: number = 0; i < n; ++i) {
      x[(n - 1) * n + i] = b[index[(n - 1)] * n + i] / a[index[n - 1] * n + (n - 1)];
      for (let j: number = n - 2; j >= 0; --j) {
        x[j * n + i] = b[index[j] * n + i];
        for (let k: number = j + 1; k < n; ++k)
          x[j * n + i] -= a[index[j] * n + k] * x[k * n + i];
        x[j * n + i] /= a[index[j] * n + j];
      }
    }
    return x;
  }

  static gaussian(a: Array<number>, index: Array<number>) {
    let n: number = index.length;
    let c = Array(n).fill(0);
    for (let i: number = 0; i < n; ++i) { index[i] = i; }
    for (let i: number = 0; i < n; ++i) {
      let c1: number = 0;
      for (let j: number = 0; j < n; ++j) {
        let c0: number = Math.abs(a[i * n + j]);
        if (c0 > c1) c1 = c0;
      }
      c[i] = c1;
    }
    let k: number = 0;
    for (let j: number = 0; j < n - 1; ++j) {
      let pi1: number = 0;
      for (let i: number = j; i < n; ++i) {
        let pi0: number = Math.abs(a[index[i] * n + j]);
        pi0 /= c[index[i]];
        if (pi0 > pi1) {
          pi1 = pi0;
          k = i;
        }
      }
      let itmp: number = index[j];
      index[j] = index[k];
      index[k] = itmp;
      for (let i: number = j + 1; i < n; ++i) {
        let pj: number = a[index[i] * n + j] / a[index[j] * n + j];
        a[index[i] * n + j] = pj;
        for (let l: number = j + 1; l < n; ++l)
          a[index[i] * n + l] -= pj * a[index[j] * n + l];
      }
    }
  }

  public static dot(vec0: Array<number>, vec1: Array<number>): number {
    let dot: number = 0;
    for (let i: number = 0; i < vec0.length; i++)  dot += vec0[i] * vec1[i];
    return dot;
  }

  static cross(a: Array<number>, b: Array<number>): Array<number> {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }

  static scalarMult(vec0: Array<number>, s: number) {
    for (let i: number = 0; i < vec0.length; i++)  vec0[i] *= s;
  }

  public static scalarMul2(vec0: Array<number>, s: number): Array<number> {
    let result: Array<number> = new Array<number>(vec0.length).fill(0);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] * s;
    return result;
  }

  public static vecAdd2(vec0: Array<number>, vec1: Array<number>): Array<number> {
    let result: Array<number> = new Array<number>(vec0.length).fill(0);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] + vec1[i];
    return result;
  }

  static vecMinus(vec0: Array<number>, vec1: Array<number>) {
    for (let i: number = 0; i < vec0.length; i++)  vec0[i] -= vec1[i];
  }

  public static vecMinus2(vec0: Array<number>, vec1: Array<number>): Array<number> {
    let result: Array<number> = new Array<number>(vec0.length).fill(0);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] - vec1[i];
    return result;
  }

  static unit(vec: Array<number>) {
    Vec.scalarMult(vec, 1.0 / Vec.magnitude(vec));
  }

  public static getCol(vec7: Array<number>, col: number, len: number): Array<number> {
    let result: Array<number> = new Array<number>(len).fill(0);
    for (let i: number = 0; i < len; i++)  result[i] = vec7[i * len + col];
    return result;
  }

  public static getRow(vec7: Array<number>, row: number, len: number): Array<number> {
    let result: Array<number> = new Array<number>(len).fill(0);
    for (let i: number = 0; i < len; i++)  result[i] = vec7[row * len + i];
    return result;
  }

  public static setRow(matrix: Array<number>, vec7: Array<number>, row: number) {
    for (let i: number = 0; i < vec7.length; i++)  matrix[row * vec7.length + i] = vec7[i];
  }

  public static multMatrix(A: Array<number>, B: Array<number>, aRows: number, aColumns: number, bRows: number, bColumns: number): Array<number> {
    if (aColumns !== bRows) throw new Error("A columns must match B rows");
    let C: Array<number> = new Array<number>(aRows * bColumns).fill(0);
    for (let i = 0; i < aRows; i++) {
      for (let j = 0; j < bColumns; j++) {
        let sum = 0;
        for (let k = 0; k < aColumns; k++) {
          sum += A[i * aColumns + k] * B[k * bColumns + j];
        }
        C[i * bColumns + j] = sum;
      }
    }
    return C;
  }

  static matrixTranspose(A: Array<number>, len: number): Array<number> {
    let result: Array<number> = new Array<number>(len * len).fill(0);
    for (let i: number = 0; i < len; i++)
      for (let j: number = 0; j < len; j++)
        result[j * len + i] = A[i * len + j];
    return result;
  }

  static matrixEye(vol: number): Array<number> {
    let result: Array<number> = new Array<number>(vol * vol).fill(0);
    for (let j: number = 0; j < vol; j++) result[j * vol + j] = 1;
    return result;
  }

  public static matrixMul(A: Array<number>, s: number): Array<number> {
    let result: Array<number> = new Array<number>(A.length).fill(0);
    for (let i: number = 0; i < A.length; i++)  result[i] = A[i] * s;
    return result;
  }

  public static matrixAdd(A: Array<number>, B: Array<number>): void {
    for (let i: number = 0; i < A.length; i++)  A[i] = A[i] + B[i];
  }
  public static matrixAdd2(A: Array<number>, B: Array<number>): Array<number> {
    let result: Array<number> = new Array<number>(A.length).fill(0);
    for (let i: number = 0; i < A.length; i++)  result[i] = A[i] + B[i];
    return result;
  }

  public static matrixDiv(A: Array<number>, s: number): void {
    for (let i: number = 0; i < A.length; i++)  A[i] = A[i] / s;
  }
  public static vectorDiv0(vec0: Array<number>, s: number): void {
    for (let i: number = 0; i < vec0.length; i++)  vec0[i] = vec0[i] / s;
  }

  public static vectorDiv(vec0: Array<number>, s: number): Array<number> {
    let result: Array<number> = new Array<number>(vec0.length).fill(0);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] / s;
    return result;
  }

  public static matVecMultiplySqr(matrix: Array<number>, vector: Array<number>, rows: number): Array<number> {
    let result: Array<number> = new Array<number>(rows).fill(0);
    for (let row: number = 0; row < rows; row++) {
      let sum: number = 0;
      for (let column: number = 0; column < rows; column++)  sum += matrix[row * rows + column] * vector[column];
      result[row] = sum;
    }
    return result;
  }

  public static vecTransMul(a: Array<number>, b: Array<number>): Array<number> {
    let result: Array<number> = new Array<number>(a.length * b.length).fill(0);
    for (let i: number = 0; i < a.length; i++)
      for (let j: number = 0; j < b.length; j++)
        result[a.length * i + j] = a[i] * b[j];
    return result;
  }

  public static vecNeg(a: Array<number>): void {
    for (let i: number = 0; i < a.length; i++) a[i] = a[i] * (-1);
  }

  public static interpolatedValue(dataArray: Float32Array, x: number, y: number, width: number): number {
    const ix: number = ~~x;
    const iy: number = ~~y;
    const dx: number = x - ix;
    const dy: number = y - iy;
    const dxdy: number = dx * dy;
    const bp: number = ix + iy * width;
    return dxdy * dataArray[bp + 1 + width] + (dy - dxdy) * dataArray[bp + width] + (dx - dxdy) * dataArray[bp + 1] + (1 - dx - dy + dxdy) * dataArray[bp];
  }

  public static solveSystem(A: Array<number>, b: Array<number>): Array<number> {
    let x = b.slice()
    for (let s = 1; s < b.length; s++) {
      for (let i = s; i < b.length; i++) {
        let a0 = A[i * b.length + s - 1] / A[(s - 1) * b.length + s - 1]
        x[i] = x[i] - a0 * x[s - 1]
        for (let j = s - 1; j < b.length; j++)
          A[i * b.length + j] -= (a0) * A[(s - 1) * b.length + j]
      }
    }
    x[b.length - 1] = x[b.length - 1] / A[(A.length - 1)]
    for (let i = b.length - 2; i >= 0; i--) {
      let buf = 0
      for (let j = b.length - 1; j > i; j--)
        buf += A[i * b.length + j] * x[j]
      x[i] = (x[i] - buf) / A[i * b.length + i]
    }
    return x;
  }
}
