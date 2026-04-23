import { test } from 'uvu';
import * as assert from 'uvu/assert';
import { SO3 } from "../SO3"

test("Data pack", () => {
    let m = new Float32Array([0.36, 0.48, -0.8, -0.8, 0.6, 0, 0.48, 0.64, 0.6]);
    let so3: SO3 = new SO3(m);
    so3.coerce();

    console.log(so3.ln());
    assert.is(0, 0)
});

test.run();