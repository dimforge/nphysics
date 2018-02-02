let m11 = *self.get_unchecked(0, 0);
let m12 = *self.get_unchecked(0, 1);
let m13 = *self.get_unchecked(0, 2);

let m21 = *self.get_unchecked(1, 0);
let m22 = *self.get_unchecked(1, 1);
let m23 = *self.get_unchecked(1, 2);

let m31 = *self.get_unchecked(2, 0);
let m32 = *self.get_unchecked(2, 1);
let m33 = *self.get_unchecked(2, 2);


let minor_m12_m23 = m22 * m33 - m32 * m23;
let minor_m11_m23 = m21 * m33 - m31 * m23;
let minor_m11_m22 = m21 * m32 - m31 * m22;

let determinant = m11 * minor_m12_m23 -
                  m12 * minor_m11_m23 +
                  m13 * minor_m11_m22;

if determinant == N::zero() {
    false
}
else {
    *self.get_unchecked_mut(0, 0) = minor_m12_m23 / determinant;
    *self.get_unchecked_mut(0, 1) = (m13 * m32 - m33 * m12) / determinant;
    *self.get_unchecked_mut(0, 2) = (m12 * m23 - m22 * m13) / determinant;

    *self.get_unchecked_mut(1, 0) = -minor_m11_m23 / determinant;
    *self.get_unchecked_mut(1, 1) = (m11 * m33 - m31 * m13) / determinant;
    *self.get_unchecked_mut(1, 2) = (m13 * m21 - m23 * m11) / determinant;

    *self.get_unchecked_mut(2, 0) = minor_m11_m22  / determinant;
    *self.get_unchecked_mut(2, 1) = (m12 * m31 - m32 * m11) / determinant;
    *self.get_unchecked_mut(2, 2) = (m11 * m22 - m21 * m12) / determinant;

    true
}
