function vout = express_vector_in_quat_frame(vin, q)
vout = rotate_vector(vin, [-q(1:3); q(4)]);
