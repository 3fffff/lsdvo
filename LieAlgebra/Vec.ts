export class Vec {
  /**
   * Calculates magnitude of a vector.
   */
  public static magnitude(vec: Float32Array): number {
    let magnitude: number = 0;
    for (let i: number = 0; i < vec.length; i++) magnitude += vec[i] * vec[i];
    return Math.sqrt(magnitude);
  }

  public static invert(a: Float32Array, len: number): Float32Array {
    let n: number = len;
    let x: Float32Array = new Float32Array(n * n);
    let b: Float32Array = new Float32Array(n * n);
    let index: Float32Array = new Float32Array(n);
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

  static gaussian(a: Float32Array, index: Float32Array) {
    let n: number = index.length;
    let c: Float32Array = new Float32Array(n);
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

  public static dot(vec0: Float32Array, vec1: Float32Array): number {
    let dot: number = 0;
    for (let i: number = 0; i < vec0.length; i++)  dot += vec0[i] * vec1[i];
    return dot;
  }

  static cross(a: Float32Array, b: Float32Array): Float32Array {
    return new Float32Array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]);
  }

  static scalarMult(vec0: Float32Array, s: number) {
    for (let i: number = 0; i < vec0.length; i++)  vec0[i] *= s;
  }

  public static scalarMult2(vec0: Float32Array, s: number): Float32Array {
    let result: Float32Array = new Float32Array(vec0.length);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] * s;
    return result;
  }

  public static vecAdd2(vec0: Float32Array, vec1: Float32Array): Float32Array {
    let result: Float32Array = new Float32Array(vec0.length);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] + vec1[i];
    return result;
  }

  static vecMinus(vec0: Float32Array, vec1: Float32Array) {
    for (let i: number = 0; i < vec0.length; i++)  vec0[i] -= vec1[i];
  }

  public static vecMinus2(vec0: Float32Array, vec1: Float32Array): Float32Array {
    let result: Float32Array = new Float32Array(vec0.length);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] - vec1[i];
    return result;
  }

  static unit(vec: Float32Array) {
    Vec.scalarMult(vec, 1.0 / Vec.magnitude(vec));
  }

  public static getCol(vec7: Float32Array, row: number, len: number): Float32Array {
    let result: Float32Array = new Float32Array(len);
    for (let i: number = 0; i < len; i++)  result[i] = vec7[i * len + row];
    return result;
  }

  public static getRow(vec7: Float32Array, row: number, len: number): Float32Array {
    let result: Float32Array = new Float32Array(len);
    for (let i: number = 0; i < len; i++)  result[i] = vec7[row * len + i];
    return result;
  }

  public static setRow(matrix: Float32Array, vec7: Float32Array, row: number) {
    for (let i: number = 0; i < vec7.length; i++)  matrix[row * vec7.length + i] = vec7[i];
  }

  public static multMatrix(A: Float32Array, B: Float32Array, aRows: number, aColumns: number, bRows: number, bColumns: number): Float32Array {
    if (aColumns !== bRows) throw new Error("A:Rows: " + aColumns + " did not match B:Columns " + bRows + ".");
    let C: Float32Array = new Float32Array(aRows * bColumns);
    for (let i: number = 0; i < aRows; i++)
      for (let j: number = 0; j < bColumns; j++)
        for (let k: number = 0; k < aColumns; k++)
          C[i * aRows + j] += A[i * aRows + k] * B[k * bRows + j];
    return C;
  }

  static matrixTranspose(A: Float32Array, len: number): Float32Array {
    let result: Float32Array = new Float32Array(len * len);
    for (let i: number = 0; i < len; i++)
      for (let j: number = 0; j < len; j++)
        result[j * len + i] = A[i * len + j];
    return result;
  }

  static matrixEye(vol: number): Float32Array {
    let result: Float32Array = new Float32Array(vol * vol);
    for (let j: number = 0; j < vol; j++) result[j * vol + j] = 1;
    return result;
  }

  public static matrixMul(A: Float32Array, s: number): Float32Array {
    let result: Float32Array = new Float32Array(A.length);
    for (let i: number = 0; i < A.length; i++)  result[i] = A[i] * s;
    return result;
  }

  public static matrixAdd(A: Float32Array, B: Float32Array): Float32Array {
    let result: Float32Array = new Float32Array(A.length);
    for (let i: number = 0; i < A.length; i++)  result[i] = A[i] + B[i];
    return result;
  }

  public static matrixDiv(A: Float32Array, s: number): Float32Array {
    let result: Float32Array = new Float32Array(A.length);
    for (let i: number = 0; i < A.length; i++)  result[i] = A[i] / s;
    return result;
  }

  public static scalarDel(vec0: Float32Array, s: number): Float32Array {
    let result: Float32Array = new Float32Array(vec0.length);
    for (let i: number = 0; i < vec0.length; i++)  result[i] = vec0[i] / s;
    return result;
  }

  public static matVecMultiplySqr(matrix: Float32Array, vector: Float32Array, rows: number): Float32Array {
    let result: Float32Array = new Float32Array(rows);
    for (let row: number = 0; row < rows; row++) {
      let sum: number = 0;
      for (let column: number = 0; column < rows; column++)  sum += matrix[row * rows + column] * vector[column];
      result[row] = sum;
    }
    return result;
  }

  public static vecT(a: Float32Array, b: Float32Array): Float32Array {
    let result: Float32Array = new Float32Array(a.length * b.length);
    for (let i: number = 0; i < a.length; i++)
      for (let j: number = 0; j < b.length; j++)
        result[a.length * i + j] = a[i] * b[j];
    return result;
  }

  public static vecNeg(a: Float32Array): Float32Array {
    let result: Float32Array = new Float32Array(a.length);
    for (let i: number = 0; i < a.length; i++) { result[i] = a[i] * (-1); }
    return result;
  }

  public static interpolatedValue(dataArray: Float32Array, x: number, y: number, width: number): number {
    let ix: number = Math.floor(x | 0);
    let iy: number = Math.floor(y | 0);
    let dx: number = x - ix;
    let dy: number = y - iy;
    let dxdy: number = dx * dy;
    let bp: number = ix + iy * width;
    return dxdy * dataArray[bp + 1 + width] + (dy - dxdy) * dataArray[bp + width] + (dx - dxdy) * dataArray[bp + 1] + (1 - dx - dy + dxdy) * dataArray[bp];
  }

  public static solveSystem(A: Float32Array, b: Float32Array): Float32Array {
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
