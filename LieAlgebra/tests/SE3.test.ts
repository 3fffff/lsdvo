import { test } from 'uvu';
import * as assert from 'uvu/assert';
import { SE3 } from "../SE3"

test("Data pack", () => {
    let vec6 = [1, 1, 1, 0.53, 0, 0];//{0.9257751770440343, 0.6696269810349835, 0.7949576278265412, 2.2639318115967644, 0.45278636231935293, 0.45278636231935293};//{1,1,1,0.5,0.1,0.1};
    let se3 = SE3.exp(vec6);
    // Test for inverse number drift
    vec6 = [-0.1687722544216055, -0.11161075918408007, -0.01406770893311903, -0.2680809945869649, 0.14390299341810078, 0.10327471107537695];//{1,1,1,0.5,0.1,0.1};
    se3 = SE3.exp(vec6);

    // Inverse test
    console.log("vec = " + SE3.ln(se3.mulSE3(se3.inverse())));

    console.log("vec = " + SE3.ln(se3));
    for (let i = 0; i < 100; i++) {
        se3 = se3.inverse();
        se3 = se3.inverse();

        console.log("vec = " + SE3.ln(se3));

        vec6 = SE3.ln(se3);
        se3 = SE3.exp(vec6);

    }
    assert.is(0, 0)
});

test.run();