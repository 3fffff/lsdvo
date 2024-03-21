import { SE3 } from "./SE3";
import { SIM3 } from "./SIM3";
import { SO3 } from "./SO3";

export class TestLie {
  SE3Test(): void {
    let vec6 = new Float32Array([1, 1, 1, 0.53, 0, 0]);//{0.9257751770440343, 0.6696269810349835, 0.7949576278265412, 2.2639318115967644, 0.45278636231935293, 0.45278636231935293};//{1,1,1,0.5,0.1,0.1};
    let se3 = SE3.exp(vec6);
    // Test for inverse number drift
    vec6 = new Float32Array([-0.1687722544216055, -0.11161075918408007, -0.01406770893311903, -0.2680809945869649, 0.14390299341810078, 0.10327471107537695]);//{1,1,1,0.5,0.1,0.1};
    se3 = SE3.exp(vec6);

    // Inverse test
    console.log("vec = " + SE3.ln(se3.mul(se3.inverse())));

    console.log("vec = " + SE3.ln(se3));
    for (let i = 0; i < 100; i++) {
      se3 = se3.inverse();
      se3 = se3.inverse();

      console.log("vec = " + SE3.ln(se3));

      vec6 = SE3.ln(se3);
      se3 = SE3.exp(vec6);

    }
  }

  SO3Test(): void {
    let m = new Float32Array([0.36, 0.48, -0.8, -0.8, 0.6, 0, 0.48, 0.64, 0.6]);
    let so3: SO3 = new SO3(m);
    so3.coerce();

    console.log(so3.ln());
  }

  SIM3Test(): void {
    let FtoC: SIM3 = SIM3.exp(new Float32Array([- 0.05822144838019558, 0.0756891620395333, -0.0022709761965872344, 0.019408896087920464, -8.390711786033482E-4, -0.017495055079029636, -0.01781943098865934])).inverse();

    console.log(SIM3.ln(FtoC.mul(FtoC.inverse())));

    console.log(SIM3.ln(FtoC));

    for (let i = 0; i < 1000; i++) {
      FtoC = FtoC.inverse().inverse();
      console.log(SIM3.ln(FtoC));
    }
  }
}