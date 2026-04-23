import { test } from 'uvu';
import * as assert from 'uvu/assert';
import { SIM3 } from '../SIM3';

test("Data pack", () => {
    let FtoC: SIM3 = SIM3.exp([- 0.05822144838019558, 0.0756891620395333, -0.0022709761965872344, 0.019408896087920464, -8.390711786033482E-4, -0.017495055079029636, -0.01781943098865934]).inverse();

    console.log(SIM3.ln(FtoC.mul(FtoC.inverse())));

    console.log(SIM3.ln(FtoC));

    for (let i = 0; i < 1000; i++) {
      FtoC = FtoC.inverse().inverse();
      console.log(SIM3.ln(FtoC));
    }
    assert.is(0, 0)
});

test.run();