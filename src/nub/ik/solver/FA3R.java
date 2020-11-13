package nub.ik.solver;

import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.List;

public class FA3R {
    /**
     * this is an adaptation of  C++ code for FA3R
     * Fast Analytical 3D Registration Method
     * Author: Jin Wu, Zebo Zhou et al.
     * e-mail: jin_wu_uestc@hotmail.com;	klinsmann.zhou@gmail.com
     * https://github.com/zarathustr/FA3R/blob/master/FA3R.cpp
     *
     * For an intuitive explanation of the problem please visit
     * http://nghiaho.com/?page_id=671
     * */


    public static void cross(double[] x, double[] y, double k, double[] z, double[] z_new){
        z_new[0] = k * (z[0] + x[1] * y[2] - x[2] * y[1]);
        z_new[1] = k * (z[1] + x[2] * y[0] - x[0] * y[2]);
        z_new[2] = k * (z[2] + x[0] * y[1] - x[1] * y[0]);
    }

    public static Quaternion FA3R(List<Vector> Q, List<Vector> P, Vector Q_centroid, Vector P_centroid){
        int n = P.size();
        if(n == 0) return new Quaternion();
        if(n == 1){
            return new Quaternion(P.get(0), Q.get(0));
        }

        double[][] sigma_ = new double[3][3];
        //float dist = 0;
        for(int i = 0; i < n; i++){
            double  w = 1.f/n;
            sigma_[0][0] += (Q.get(i).x() - Q_centroid.x()) * (P.get(i).x() - P_centroid.x()) * w;
            sigma_[0][1] += (Q.get(i).x() - Q_centroid.x()) * (P.get(i).y() - P_centroid.y()) * w;
            sigma_[0][2] += (Q.get(i).x() - Q_centroid.x()) * (P.get(i).z() - P_centroid.z()) * w;
            sigma_[1][0] += (Q.get(i).y() - Q_centroid.y()) * (P.get(i).x() - P_centroid.x()) * w;
            sigma_[1][1] += (Q.get(i).y() - Q_centroid.y()) * (P.get(i).y() - P_centroid.y()) * w;
            sigma_[1][2] += (Q.get(i).y() - Q_centroid.y()) * (P.get(i).z() - P_centroid.z()) * w;
            sigma_[2][0] += (Q.get(i).z() - Q_centroid.z()) * (P.get(i).x() - P_centroid.x()) * w;
            sigma_[2][1] += (Q.get(i).z() - Q_centroid.z()) * (P.get(i).y() - P_centroid.y()) * w;
            sigma_[2][2] += (Q.get(i).z() - Q_centroid.z()) * (P.get(i).z() - P_centroid.z()) * w;
            //dist += Vector.distance(P.get(i).normalize(new Vector()), Q.get(i).normalize(new Vector()));
        }
        //if(dist / n < 0.01) return new Quaternion();


        double[] hx = new double[]{sigma_[0][0], sigma_[1][0], sigma_[2][0]};
        double[] hy = new double[]{sigma_[0][1], sigma_[1][1], sigma_[2][1]};
        double[] hz = new double[]{sigma_[0][2], sigma_[1][2], sigma_[2][2]};

        double[] hx_ = new double[3];
        double[] hy_ = new double[3];
        double[] hz_ = new double[3];

        double k;

        for(int i = 0; i < 10; ++i)
        {
            k = 2.0 / (hx[0] * hx[0] + hx[1] * hx[1] + hx[2] * hx[2] +
                    hy[0] * hy[0] + hy[1] * hy[1] + hy[2] * hy[2] +
                    hz[0] * hz[0] + hz[1] * hz[1] + hz[2] * hz[2] + 1.0);

            hx_[0] = hx[0]; hx_[1] = hx[1]; hx_[2] = hx[2];
            hy_[0] = hy[0]; hy_[1] = hy[1]; hy_[2] = hy[2];
            hz_[0] = hz[0]; hz_[1] = hz[1]; hz_[2] = hz[2];

            cross(hx_, hy_, k, hz_, hz);
            cross(hz_, hx_, k, hy_, hy);
            cross(hy_, hz_, k, hx_, hx);
        }

        Vector x = new Vector((float) hx[0], (float) hx[1], (float) hx[2]);
        Vector y = new Vector((float) hy[0], (float) hy[1], (float) hy[2]);
        Vector z = new Vector((float) hz[0], (float) hz[1], (float) hz[2]);

        //Rotation
        Quaternion q = new Quaternion();
        q.fromRotatedBasis(x, y, z);
        q.normalize();
        return q;
    }
}
