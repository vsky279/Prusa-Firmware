// Exact original fitter and reconstruction, commit 24fab9457edab4273ed608fe0c233b08017b2637.
BedSkewOffsetDetectionResultType old_fitter(
    const float  *measured_pts,
    uint8_t       npts,
    const float  *true_pts,
    float        *vec_x,
    float        *vec_y,
    float        *cntr,
    int8_t
#ifdef SUPPORT_VERBOSITY
    verbosity_level
#endif //SUPPORT_VERBOSITY
    )
{
	float angleDiff;
	#ifdef SUPPORT_VERBOSITY
    if (verbosity_level >= 10) {
		SERIAL_ECHOLNPGM("calculate machine skew and offset LS");

        // Show the initial state, before the fitting.
        SERIAL_ECHOPGM("X vector, initial: ");
        MYSERIAL.print(vec_x[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(vec_x[1], 5);
        SERIAL_ECHOLNPGM("");

        SERIAL_ECHOPGM("Y vector, initial: ");
        MYSERIAL.print(vec_y[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(vec_y[1], 5);
        SERIAL_ECHOLNPGM("");

        SERIAL_ECHOPGM("center, initial: ");
        MYSERIAL.print(cntr[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(cntr[1], 5);
        SERIAL_ECHOLNPGM("");

        for (uint8_t i = 0; i < npts; ++i) {
            SERIAL_ECHOPGM("point #");
            MYSERIAL.print(int(i));
            SERIAL_ECHOPGM(" measured: (");
            MYSERIAL.print(measured_pts[i * 2], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(measured_pts[i * 2 + 1], 5);
            SERIAL_ECHOPGM("); target: (");
            MYSERIAL.print(pgm_read_float(true_pts + i * 2), 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(pgm_read_float(true_pts + i * 2 + 1), 5);
            SERIAL_ECHOPGM("), error: ");
            MYSERIAL.print(sqrt(
                sqr(pgm_read_float(true_pts + i * 2) - measured_pts[i * 2]) +
                sqr(pgm_read_float(true_pts + i * 2 + 1) - measured_pts[i * 2 + 1])), 5);
            SERIAL_ECHOLNPGM("");
        }
        delay_keep_alive(100);
    }
	#endif // SUPPORT_VERBOSITY

    // Run some iterations of the Gauss-Newton method of non-linear least squares.
    // Initial set of parameters:
    // X,Y offset
    cntr[0] = 0.f;
    cntr[1] = 0.f;
    // Rotation of the machine X axis from the bed X axis.
    float a1 = 0;
    // Rotation of the machine Y axis from the bed Y axis.
    float a2 = 0;
    for (int8_t iter = 0; iter < 100; ++iter) {
        float c1 = cos(a1) * MACHINE_AXIS_SCALE_X;
        float s1 = sin(a1) * MACHINE_AXIS_SCALE_X;
        float c2 = cos(a2) * MACHINE_AXIS_SCALE_Y;
        float s2 = sin(a2) * MACHINE_AXIS_SCALE_Y;
        // Prepare the Normal equation for the Gauss-Newton method.
        float A[4][4] = { 0.f };
        float b[4] = { 0.f };
        float acc;
		delay_keep_alive(0); //manage heater, reset watchdog, manage inactivity
        for (uint8_t r = 0; r < 4; ++r) {
            for (uint8_t c = 0; c < 4; ++c) {
                acc = 0;
                // J^T times J
                for (uint8_t i = 0; i < npts; ++i) {
                    // First for the residuum in the x axis:
                    if (r != 1 && c != 1) {
                        float a = 
                             (r == 0) ? 1.f :
                            ((r == 2) ? (-s1 * measured_pts[2 * i]) :
                                        (-c2 * measured_pts[2 * i + 1]));
                        float b = 
                             (c == 0) ? 1.f :
                            ((c == 2) ? (-s1 * measured_pts[2 * i]) :
                                        (-c2 * measured_pts[2 * i + 1]));
                        float w = point_weight_x(i, measured_pts[2 * i + 1]);
                        acc += a * b * w;
                    }
                    // Second for the residuum in the y axis. 
                    // The first row of the points have a low weight, because their position may not be known
                    // with a sufficient accuracy.
                    if (r != 0 && c != 0) {
                        float a = 
                             (r == 1) ? 1.f :
                            ((r == 2) ? ( c1 * measured_pts[2 * i]) :
                                        (-s2 * measured_pts[2 * i + 1]));
                        float b = 
                             (c == 1) ? 1.f :
                            ((c == 2) ? ( c1 * measured_pts[2 * i]) :
                                        (-s2 * measured_pts[2 * i + 1]));
                        float w = point_weight_y(i, measured_pts[2 * i + 1]);
                        acc += a * b * w;
                    }
                }
                A[r][c] = acc;
            }
            // J^T times f(x)
            acc = 0.f;
            for (uint8_t i = 0; i < npts; ++i) {
                {
                    float j = 
                         (r == 0) ? 1.f :
                        ((r == 1) ? 0.f :
                        ((r == 2) ? (-s1 * measured_pts[2 * i]) :
                                    (-c2 * measured_pts[2 * i + 1])));
                    float fx = c1 * measured_pts[2 * i] - s2 * measured_pts[2 * i + 1] + cntr[0] - pgm_read_float(true_pts + i * 2);
                    float w = point_weight_x(i, measured_pts[2 * i + 1]);
                    acc += j * fx * w;
                }
                {
                    float j = 
                         (r == 0) ? 0.f :
                        ((r == 1) ? 1.f :
                        ((r == 2) ? ( c1 * measured_pts[2 * i]) :
                                    (-s2 * measured_pts[2 * i + 1])));
                    float fy = s1 * measured_pts[2 * i] + c2 * measured_pts[2 * i + 1] + cntr[1] - pgm_read_float(true_pts + i * 2 + 1);
                    float w = point_weight_y(i, measured_pts[2 * i + 1]);
                    acc += j * fy * w;
                }
            }
            b[r] = -acc;
        }

        // Solve for h by a Gauss iteration method.
        float h[4] = { 0.f };
        for (uint8_t gauss_iter = 0; gauss_iter < 100; ++gauss_iter) {
            h[0] = (b[0] - A[0][1] * h[1] - A[0][2] * h[2] - A[0][3] * h[3]) / A[0][0];
            h[1] = (b[1] - A[1][0] * h[0] - A[1][2] * h[2] - A[1][3] * h[3]) / A[1][1];
            h[2] = (b[2] - A[2][0] * h[0] - A[2][1] * h[1] - A[2][3] * h[3]) / A[2][2];
            h[3] = (b[3] - A[3][0] * h[0] - A[3][1] * h[1] - A[3][2] * h[2]) / A[3][3];
        }

        // and update the current position with h.
        // It may be better to use the Levenberg-Marquart method here,
        // but because we are very close to the solution alread,
        // the simple Gauss-Newton non-linear Least Squares method works well enough.
        cntr[0] += h[0];
        cntr[1] += h[1];
        a1 += h[2];
        a2 += h[3];

		#ifdef SUPPORT_VERBOSITY
        if (verbosity_level >= 20) {
            SERIAL_ECHOPGM("iteration: ");
            MYSERIAL.print(int(iter));
			SERIAL_ECHOPGM("; correction vector: ");
            MYSERIAL.print(h[0], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(h[1], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(h[2], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(h[3], 5);
            SERIAL_ECHOLNPGM("");
            SERIAL_ECHOPGM("corrected x/y: ");
            MYSERIAL.print(cntr[0], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(cntr[0], 5);
            SERIAL_ECHOLNPGM("");
            SERIAL_ECHOPGM("corrected angles: ");
            MYSERIAL.print(180.f * a1 / M_PI, 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(180.f * a2 / M_PI, 5);
            SERIAL_ECHOLNPGM("");
        }
		#endif // SUPPORT_VERBOSITY
    }

    vec_x[0] =  cos(a1) * MACHINE_AXIS_SCALE_X;
    vec_x[1] =  sin(a1) * MACHINE_AXIS_SCALE_X;
    vec_y[0] = -sin(a2) * MACHINE_AXIS_SCALE_Y;
    vec_y[1] =  cos(a2) * MACHINE_AXIS_SCALE_Y;

    BedSkewOffsetDetectionResultType result = BED_SKEW_OFFSET_DETECTION_PERFECT;
    {
        angleDiff = fabs(a2 - a1);
        /// XY skew and Y-bed skew
        DBG(_n("Measured skews: %f %f\n"), degrees(a2 - a1), degrees(a2));
        eeprom_update_float_notify((float *)(EEPROM_XYZ_CAL_SKEW), angleDiff); //storing xyz cal. skew to be able to show in support menu later
        if (angleDiff > bed_skew_angle_mild)
            result = (angleDiff > bed_skew_angle_extreme) ?
                BED_SKEW_OFFSET_DETECTION_SKEW_EXTREME :
                BED_SKEW_OFFSET_DETECTION_SKEW_MILD;
        if (fabs(a1) > bed_skew_angle_extreme ||
            fabs(a2) > bed_skew_angle_extreme)
            result = BED_SKEW_OFFSET_DETECTION_SKEW_EXTREME;
    }
	#ifdef SUPPORT_VERBOSITY
    if (verbosity_level >= 1) {
        SERIAL_ECHOPGM("correction angles: ");
        MYSERIAL.print(180.f * a1 / M_PI, 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(180.f * a2 / M_PI, 5);
        SERIAL_ECHOLNPGM("");
    }

    if (verbosity_level >= 10) {
        // Show the adjusted state, before the fitting.
        SERIAL_ECHOPGM("X vector new, inverted: ");
        MYSERIAL.print(vec_x[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(vec_x[1], 5);
        SERIAL_ECHOLNPGM("");

        SERIAL_ECHOPGM("Y vector new, inverted: ");
        MYSERIAL.print(vec_y[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(vec_y[1], 5);
        SERIAL_ECHOLNPGM("");

        SERIAL_ECHOPGM("center new, inverted: ");
        MYSERIAL.print(cntr[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(cntr[1], 5);
        SERIAL_ECHOLNPGM("");
        delay_keep_alive(100);

        SERIAL_ECHOLNPGM("Error after correction: ");
    }
	#endif // SUPPORT_VERBOSITY
    // Measure the error after correction.
    for (uint8_t i = 0; i < npts; ++i) {
        float x = vec_x[0] * measured_pts[i * 2] + vec_y[0] * measured_pts[i * 2 + 1] + cntr[0];
        float y = vec_x[1] * measured_pts[i * 2] + vec_y[1] * measured_pts[i * 2 + 1] + cntr[1];
        float errX = pgm_read_float(true_pts + i * 2) - x;
        float errY = pgm_read_float(true_pts + i * 2 + 1) - y;
        float err = hypot(errX, errY);
		#ifdef SUPPORT_VERBOSITY
		if (verbosity_level >= 10) {
			SERIAL_ECHOPGM("point #");
			MYSERIAL.print(int(i));
			SERIAL_ECHOLNPGM(":");
		}
		#endif // SUPPORT_VERBOSITY

		if (point_on_1st_row(i)) {
				#ifdef SUPPORT_VERBOSITY
				if(verbosity_level >= 20) SERIAL_ECHOPGM("Point on first row");
				#endif // SUPPORT_VERBOSITY
				float w = point_weight_y(i, measured_pts[2 * i + 1]);
				if (errX > BED_CALIBRATION_POINT_OFFSET_MAX_1ST_ROW_X ||
					(w != 0.f && errY > BED_CALIBRATION_POINT_OFFSET_MAX_1ST_ROW_Y)) {
					result = BED_SKEW_OFFSET_DETECTION_FITTING_FAILED;
					#ifdef SUPPORT_VERBOSITY
					if (verbosity_level >= 20) {
						SERIAL_ECHOPGM(", weigth Y: ");
						MYSERIAL.print(w);
						if (errX > BED_CALIBRATION_POINT_OFFSET_MAX_1ST_ROW_X) SERIAL_ECHOPGM(", error X > max. error X");
						if (w != 0.f && errY > BED_CALIBRATION_POINT_OFFSET_MAX_1ST_ROW_Y) SERIAL_ECHOPGM(", error Y > max. error Y");
					}
					#endif // SUPPORT_VERBOSITY
				}
		}
		else {
			#ifdef SUPPORT_VERBOSITY
			if(verbosity_level >=20 ) SERIAL_ECHOPGM("Point not on first row");
			#endif // SUPPORT_VERBOSITY
			if (err > BED_CALIBRATION_POINT_OFFSET_MAX_EUCLIDIAN) {
				result = BED_SKEW_OFFSET_DETECTION_FITTING_FAILED;
				#ifdef SUPPORT_VERBOSITY
				if(verbosity_level >= 20) SERIAL_ECHOPGM(", error > max. error euclidian"); 
				#endif // SUPPORT_VERBOSITY
			}
        }
		#ifdef SUPPORT_VERBOSITY
        if (verbosity_level >= 10) {
			SERIAL_ECHOLNPGM("");
            SERIAL_ECHOPGM("measured: (");
            MYSERIAL.print(measured_pts[i * 2], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(measured_pts[i * 2 + 1], 5);
            SERIAL_ECHOPGM("); corrected: (");
            MYSERIAL.print(x, 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(y, 5);
            SERIAL_ECHOPGM("); target: (");
            MYSERIAL.print(pgm_read_float(true_pts + i * 2), 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(pgm_read_float(true_pts + i * 2 + 1), 5);
			SERIAL_ECHOLNPGM(")");
			SERIAL_ECHOPGM("error: ");
            MYSERIAL.print(err);
			SERIAL_ECHOPGM(", error X: ");
			MYSERIAL.print(errX);
			SERIAL_ECHOPGM(", error Y: ");
			MYSERIAL.print(errY);
			SERIAL_ECHOLNPGM("");
			SERIAL_ECHOLNPGM("");
        }
		#endif // SUPPORT_VERBOSITY
    }
	#ifdef SUPPORT_VERBOSITY
	if (verbosity_level >= 20) {
		SERIAL_ECHOLNPGM("Max. errors:");
		SERIAL_ECHOPGM("Max. error X:");
		MYSERIAL.println(BED_CALIBRATION_POINT_OFFSET_MAX_1ST_ROW_X);
		SERIAL_ECHOPGM("Max. error Y:");
		MYSERIAL.println(BED_CALIBRATION_POINT_OFFSET_MAX_1ST_ROW_Y);
		SERIAL_ECHOPGM("Max. error euclidian:");
		MYSERIAL.println(BED_CALIBRATION_POINT_OFFSET_MAX_EUCLIDIAN);
		SERIAL_ECHOLNPGM("");
	}
	#endif // SUPPORT_VERBOSITY

    #if 0
    if (result == BED_SKEW_OFFSET_DETECTION_PERFECT && fabs(a1) < bed_skew_angle_mild && fabs(a2) < bed_skew_angle_mild) {
		#ifdef SUPPORT_VERBOSITY
		if (verbosity_level > 0)
            SERIAL_ECHOLNPGM("Very little skew detected. Disabling skew correction.");
		#endif // SUPPORT_VERBOSITY
        // Just disable the skew correction.
        vec_x[0] = MACHINE_AXIS_SCALE_X;
        vec_x[1] = 0.f;
        vec_y[0] = 0.f;
        vec_y[1] = MACHINE_AXIS_SCALE_Y;
    }
    #else
    if (result == BED_SKEW_OFFSET_DETECTION_PERFECT) {
		#ifdef SUPPORT_VERBOSITY
		if (verbosity_level > 0)
            SERIAL_ECHOLNPGM("Very little skew detected. Orthogonalizing the axes.");
		#endif // SUPPORT_VERBOSITY
		// Orthogonalize the axes.
        a1 = 0.5f * (a1 + a2);
        vec_x[0] =  cos(a1) * MACHINE_AXIS_SCALE_X;
        vec_x[1] =  sin(a1) * MACHINE_AXIS_SCALE_X;
        vec_y[0] = -sin(a1) * MACHINE_AXIS_SCALE_Y;
        vec_y[1] =  cos(a1) * MACHINE_AXIS_SCALE_Y;
        // Refresh the offset.
        cntr[0] = 0.f;
        cntr[1] = 0.f;
        float wx = 0.f;
        float wy = 0.f;
        for (int8_t i = 0; i < npts; ++ i) {
            float x = vec_x[0] * measured_pts[i * 2] + vec_y[0] * measured_pts[i * 2 + 1];
            float y = vec_x[1] * measured_pts[i * 2] + vec_y[1] * measured_pts[i * 2 + 1];
            float w = point_weight_x(i, y);
			cntr[0] += w * (pgm_read_float(true_pts + i * 2) - x);
			wx += w;
			#ifdef SUPPORT_VERBOSITY
			if (verbosity_level >= 20) {
				MYSERIAL.print(i);
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("Weight_x:");
				MYSERIAL.print(w);
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("cntr[0]:");
				MYSERIAL.print(cntr[0]);
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("wx:");
				MYSERIAL.print(wx);
			}
			#endif // SUPPORT_VERBOSITY
            w = point_weight_y(i, y);
			cntr[1] += w * (pgm_read_float(true_pts + i * 2 + 1) - y);
			wy += w;
			#ifdef SUPPORT_VERBOSITY
			if (verbosity_level >= 20) {
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("Weight_y:");
				MYSERIAL.print(w);
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("cntr[1]:");
				MYSERIAL.print(cntr[1]);
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("wy:");
				MYSERIAL.print(wy);
				SERIAL_ECHOLNPGM("");
				SERIAL_ECHOLNPGM("");
			}
			#endif // SUPPORT_VERBOSITY

		}
        cntr[0] /= wx;
        cntr[1] /= wy;
		#ifdef SUPPORT_VERBOSITY
		if (verbosity_level >= 20) {
			SERIAL_ECHOLNPGM("");
			SERIAL_ECHOLNPGM("Final cntr values:");
			SERIAL_ECHOLNPGM("cntr[0]:");
			MYSERIAL.print(cntr[0]);
			SERIAL_ECHOLNPGM("");
			SERIAL_ECHOLNPGM("cntr[1]:");
			MYSERIAL.print(cntr[1]);
			SERIAL_ECHOLNPGM("");
		}
		#endif // SUPPORT_VERBOSITY
    }
    #endif

    // Invert the transformation matrix made of vec_x, vec_y and cntr.
    {
        float d = vec_x[0] * vec_y[1] - vec_x[1] * vec_y[0];
        float Ainv[2][2] = {
            { vec_y[1] / d, -vec_y[0] / d },
            { -vec_x[1] / d, vec_x[0] / d }
        };
        float cntrInv[2] = {
            -Ainv[0][0] * cntr[0] - Ainv[0][1] * cntr[1],
            -Ainv[1][0] * cntr[0] - Ainv[1][1] * cntr[1]
        };
        vec_x[0] = Ainv[0][0];
        vec_x[1] = Ainv[1][0];
        vec_y[0] = Ainv[0][1];
        vec_y[1] = Ainv[1][1];
        cntr[0] = cntrInv[0];
        cntr[1] = cntrInv[1];
    }
	#ifdef SUPPORT_VERBOSITY
    if (verbosity_level >= 1) {
        // Show the adjusted state, before the fitting.
        SERIAL_ECHOPGM("X vector, adjusted: ");
        MYSERIAL.print(vec_x[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(vec_x[1], 5);
        SERIAL_ECHOLNPGM("");

        SERIAL_ECHOPGM("Y vector, adjusted: ");
        MYSERIAL.print(vec_y[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(vec_y[1], 5);
        SERIAL_ECHOLNPGM("");

        SERIAL_ECHOPGM("center, adjusted: ");
        MYSERIAL.print(cntr[0], 5);
        SERIAL_ECHOPGM(", ");
        MYSERIAL.print(cntr[1], 5);
        SERIAL_ECHOLNPGM("");
        delay_keep_alive(100);
    }

    if (verbosity_level >= 2) {
        SERIAL_ECHOLNPGM("Difference after correction: ");
        for (uint8_t i = 0; i < npts; ++i) {
            float x = vec_x[0] * pgm_read_float(true_pts + i * 2) + vec_y[0] * pgm_read_float(true_pts + i * 2 + 1) + cntr[0];
            float y = vec_x[1] * pgm_read_float(true_pts + i * 2) + vec_y[1] * pgm_read_float(true_pts + i * 2 + 1) + cntr[1];
            SERIAL_ECHOPGM("point #");
            MYSERIAL.print(int(i));
            SERIAL_ECHOPGM("measured: (");
            MYSERIAL.print(measured_pts[i * 2], 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(measured_pts[i * 2 + 1], 5);
            SERIAL_ECHOPGM("); measured-corrected: (");
            MYSERIAL.print(x, 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(y, 5);
            SERIAL_ECHOPGM("); target: (");
            MYSERIAL.print(pgm_read_float(true_pts + i * 2), 5);
            SERIAL_ECHOPGM(", ");
            MYSERIAL.print(pgm_read_float(true_pts + i * 2 + 1), 5);
            SERIAL_ECHOPGM("), error: ");
            MYSERIAL.print( hypot(measured_pts[i * 2] - x, measured_pts[i * 2 + 1] - y) );
            SERIAL_ECHOLNPGM("");
        }
		if (verbosity_level >= 20) {
			SERIAL_ECHOLNPGM("");
			SERIAL_ECHOLNPGM("Calculate offset and skew returning result:");
			MYSERIAL.print(int(result));
			SERIAL_ECHOLNPGM("");
			SERIAL_ECHOLNPGM("");
		}
        delay_keep_alive(100);
    }
	#endif // SUPPORT_VERBOSITY


    return result;
}
static void old_reconstruction(float *pts) {
        float x1 = pts[0];
        float y1 = pts[1];
        float x2 = pts[2];
        float y2 = pts[3];
        float d2 = pow(pts[6]-pts[0], 2) + pow(pts[7]-pts[1], 2);
        pts[4] = (sqrt(d2*pow(y1 - y2, 2)*(pow(x1, 2) - 2*x2*x1 + pow(x2, 2) + pow(y1 - y2, 2))) + x2*pow(y1 - y2, 2) + pow(x2, 3) - 2*x1*pow(x2, 2) + pow(x1, 2)*x2)/
            (pow(x1, 2) - 2*x2*x1 + pow(x2, 2) + pow(y1 - y2, 2));
        pts[5] = -(x2-x1)/(y2-y1)*(pts[4]-x2)+y2;
        
		
}
