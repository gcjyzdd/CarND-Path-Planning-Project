#include "helpers.h"

#include "Dense"
#include <algorithm> // std::copy
#include <random>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static const int N_SAMPLES = 1;
static std::random_device rd{};
static std::mt19937 gen{rd()};


Traj2D
Vehicle::choose_next_state_v3(const vector<Vehicle> &predictions)
{
    size_t nd = 3, nv = 30;

    vector<Traj2D> trajectories;

    double s0 = state[0];
    double d0 = state[3];
    double v0 = state[1];

    int idx = 0, r_idx = 0;
    // check if there is a vehicle ahead of the host vehicle,
    // if yes, get the index of the target vehicle
    bool ahead = get_vehicle_ahead(predictions, idx);

    vector<double> vt(nv + 1);
    for (size_t j = 0; j < nv; ++j)
    {
        vt[j] = float(j) / (nv - 1) * MAX_SPEED;
    }
    vector<bool> lc_valid(nd);
    for (size_t i = 0; i < nd; ++i)
    {
        lc_valid[i] = false;
        int side_v_id = 0;
        if (i != lane)
        {
            lc_valid[i] = check_lane_change(predictions, i, side_v_id);
        }
    }
    // if cross two lanes, make sure both lanes are empty
    if (lane == 0)
        lc_valid[2] = lc_valid[2] && lc_valid[1];
    if (lane == 2)
        lc_valid[0] = lc_valid[0] && lc_valid[1];

    for (size_t i = 0; i < nd; ++i)
    {
        if (i == lane)
        { // going straight
            if (ahead)
            { // follow front vehicle
                double fv_speed = predictions[idx].state[1];
                double dist = predictions[idx].state[0] - s0;
                double ref_speed = 0;
                cout << "Dist to fv = " << dist << "m"
                     << ", fv speed = " << fv_speed << "m/s"
                     << ", v0 = " << v0 << "m/s\n";
                if (dist > 25.0)
                { // far from front vehicle, increase speed
                    ref_speed = 0.08 * (dist - 10.) + fv_speed;
                }
                else if (dist < 20.0)
                { // too close, decrease speed
                    ref_speed = fv_speed / 8. * (dist - 10) + fv_speed;
                }
                else
                { // keep distance
                    ref_speed = fv_speed;
                }
                // clipping speed
                ref_speed = ref_speed > MAX_SPEED ? MAX_SPEED : ref_speed;
                ref_speed = ref_speed > 0 ? ref_speed : 0;
                vt[nv] = ref_speed;
                cout << "Going straight, ref_speed = " << ref_speed << "m/s\n";
                for (size_t j = 0; j <= nv; ++j)
                {
                    trajectories.push_back(Traj2D(s0, d0, v0, vt[j], i));
                }
                r_idx = trajectories.size() - 1;
            }
            else
            { // going as fast as possible
                trajectories.push_back(Traj2D(s0, d0, v0, MAX_SPEED, i));
            }
        }
        else
        { // lane change
            if (lc_valid[i])
            {
                for (size_t j = 0; j < nv; ++j)
                {
                    trajectories.push_back(Traj2D(s0, d0, v0, vt[j], i));
                }
            }
        }
    }
    double cost, best_cost = 1e9;
    int index = 0, best_index = 0;
    for (index = 0; index < trajectories.size(); ++index)
    {
        cost = calculate_cost_veh_traj(trajectories[index], HORIZON,
                                       predictions);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_index = index;
        }
        //cout << cost << ", ";
    }
    //cout << endl;
    cout << "Best cost = " << best_cost << ", best_index = " << best_index
         << endl;
    calculate_cost_veh_traj(trajectories[best_index], HORIZON,
                            predictions, true);
    //calculate_cost_veh_traj(trajectories[r_idx], HORIZON,
    //                        predictions, true);
    target_speed = trajectories[best_index].vs;
    return trajectories[best_index];
}

/**
 * check if there is a vehicle ahead of the host vehicle,
 * if yes, get the index of the target vehicle
*/
bool Vehicle::get_vehicle_ahead(const vector<Vehicle> &predictions, int &idx)
{
    bool found = false;
    double d = 1e9;

    for (size_t i = 0; i < predictions.size(); ++i)
    {
        if (getLane(predictions[i].state[3]) == lane)
        {
            if (this->state[0] < predictions[i].state[0])
            {
                if (predictions[i].state[0] < d)
                {
                    idx = i;
                    d = predictions[i].state[0];
                }
                found = true;
            }
        }
    }
    return found;
}

/**
 * Check if it's available to change lane.
 * 
 * Briefly speaking, the host vehicle should change lane either if there is no vehicles at 
 * its side, or the side vehicle is behind the host vehicle and the host vehicle is driving
 * faster than the side vehicle
*/
bool Vehicle::check_lane_change(const vector<Vehicle> &predictions, const int lane_in, int &idx)
{
    // enable lane change
    bool lc = true;
    double d = 1e9;
    double L1 = 10 + state[0], L2 = state[0] - 3, L3 = state[0] - 5;
    for (size_t i = 0; i < predictions.size(); ++i)
    {
        double t_s = predictions[i].state[0];
        double t_v = predictions[i].state[1];
        double t_d = predictions[i].state[3];
        if (getLane(t_d) == lane_in)
        { // target vehicle in the same lane as lane_in
            if ((t_s > L3) && (t_s < L1))
            { // the target vehicle is interesting
                cout << "Find a vehicle: "
                     << "host s = " << state[0] << " d = " << state[3] << " v = " << state[1] << ", target s = " << t_s << " d = " << t_d << " v = " << t_v << endl;
                if ((t_s > L2) && (t_s < L1))
                { // the target vehicle is too close to the host vehicle, do not change lane
                    lc = false;
                    return lc;
                }
                else if ((t_s > L3) && (t_s < L2))
                { // the target vehicle is a bit behind the host vehicle
                    if (this->state[1] < t_v)
                    { // but the target vehicle is driving faster. Do not change lane in this case
                        if (predictions[i].state[0] < d)
                        {
                            d = predictions[i].state[0];
                            idx = i;
                        }
                        lc = false;
                        return lc;
                    }
                }
            }
        }
    }
    cout << "Free to change to lane " << lane_in << endl;
    return lc;
}

/**
 * Path trajectory generation
 * 
 * Given initial state [s0,v0,a0], destination state [sT,vT,aT], and 
 * time duration T, generate an optimal trajectory.
*/
vector<double> PTG(const vector<double> &start_s, const vector<double> &start_d,
                   const int &target_vehicle, const vector<double> &delta,
                   const double &T, const vector<Vehicle> &predictions)
{
    Vehicle target = predictions[target_vehicle];
    vector<vector<double>> all_goals; // s,d,t
    float timestep = 0.5;
    float t = T - 4 * timestep;
    while (t <= (T + 4 * timestep))
    {
        vector<double> target_state = VecAdd(target.state_in(t), delta);
        vector<double> goals = target_state;
        goals.push_back(t);
        all_goals.push_back(goals);
        for (int i = 0; i < N_SAMPLES; ++i)
        {
            // cout << " " << goals.size();
            goals = perturb_goal(target_state);
            goals.push_back(t);
            all_goals.push_back(goals);
        }
        // cout << " \n";
        t += timestep;
    }
    // cout << "perturb ..." << all_goals.size() << "\n";
    vector<vector<double>> trajectories;
    vector<double> traj(13);
    for (size_t i = 0; i < all_goals.size(); ++i)
    {
        vector<double> s_goal(all_goals[i].begin(), all_goals[i].begin() + 3);
        vector<double> d_goal(all_goals[i].begin() + 3, all_goals[i].begin() + 6);
        double t = all_goals[i][6];
        vector<double> s_coeffs = JMT(start_s, s_goal, t);
        vector<double> d_coeffs = JMT(start_d, d_goal, t);
        copy(s_coeffs.begin(), s_coeffs.end(), traj.begin());
        copy(d_coeffs.begin(), d_coeffs.end(), traj.begin() + 6);
        traj[12] = t;
        trajectories.push_back(traj);
        // cout << "perturb ..." << i;
    }
    vector<double> costs;
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        costs.push_back(
            calculate_cost(trajectories[i], target_vehicle, delta, T, predictions));
        // costs.push_back(i);
    }
    int idx = distance(costs.begin(), min_element(costs.begin(), costs.end()));
    calculate_cost(trajectories[idx], target_vehicle, delta, T, predictions,
                   true);
    vector<double> rst;
    rst = trajectories[idx];
    rst.push_back(costs[idx]);

    return rst;
}

vector<double> PTG_free(const vector<double> &start,
                        const vector<double> &target, const double &T)
{
    vector<double> s_start(start.begin(), start.begin() + 3);
    vector<double> d_start(start.begin() + 3, start.begin() + 6);

    vector<double> s_goal(target.begin(), target.begin() + 3);
    vector<double> d_goal(target.begin() + 3, target.begin() + 6);

    vector<double> s_coeffs = JMT(s_start, s_goal, T);
    vector<double> d_coeffs = JMT(d_start, d_goal, T);

    auto rst = s_coeffs;
    rst.insert(rst.end(), d_coeffs.begin(), d_coeffs.end());
    rst.push_back(T);
    rst.push_back(0.);

    vector<double> delta;
    vector<Vehicle> preds;
    cout << "acc cost = " << max_accel_cost(rst, 0, delta, 0, preds) << endl;

    return rst;
}

vector<double> JMT(vector<double> start, vector<double> end, double T)
{
    /*
Calculate the Jerk Minimizing Trajectory that connects the initial state
to the final state in time T.

INPUTS

start - the vehicles start location given as a length three array
corresponding to initial values of [s, s_dot, s_double_dot]

end   - the desired end state for vehicle. Like "start" this is a
length three array.

T     - The duration, in seconds, over which this maneuver should occur.

OUTPUT
an array of length 6, each value corresponding to a coefficent in the
polynomial s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 *
t**5

EXAMPLE

> JMT( [0, 10, 0], [10, 10, 0], 1)
[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
*/
    vector<double> coeffs(6);
    coeffs[0] = start[0];
    coeffs[1] = start[1];
    coeffs[2] = start[2] * 0.5;

    double t2 = T * T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;

    MatrixXd A(3, 3);
    A << t3, t4, t5, 3 * t2, 4 * t3, 5 * t4, 6 * T, 12 * t2, 20 * t3;
    VectorXd b(3);
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2),
        end[1] - (start[1] + start[2] * T), end[2] - start[2];
    VectorXd x = A.colPivHouseholderQr().solve(b);
    for (int i = 0; i < 3; ++i)
    {
        coeffs[3 + i] = x[i];
    }
    return coeffs;
}

vector<double> VecAdd(const vector<double> &v1, const vector<double> &v2)
{
    if (v1.size() != v2.size())
    {
        cerr << "Invalid input\n";
    }
    vector<double> r;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        r.push_back(v1[i] + v2[i]);
    }
    return r;
}

vector<double> VecSub(const vector<double> &v1, const vector<double> &v2)
{
    if (v1.size() != v2.size())
    {
        cerr << "Invalid input\n";
    }
    vector<double> r;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        r.push_back(v1[i] - v2[i]);
    }
    return r;
}

double logistic(double x) { return (2.0 / (1. + exp(-abs(x))) - 1.); }

vector<double> perturb_goal(const vector<double> &sd)
{
    vector<double> r;
    for (size_t i = 0; i < sd.size(); ++i)
    {
        std::normal_distribution<> d(sd[i], SIGMA_SD[i]);
        r.push_back(d(gen));
    }
    return r;
}

double polyval(const vector<double> &coeffs, double x)
{
    double s = 0.;

    for (size_t i = 0; i < coeffs.size(); ++i)
    {
        s += coeffs[i] * pow(x, i);
    }
    return s;
}

vector<double> polyval(const vector<double> &coeffs, const vector<double> &x)
{
    vector<double> r;
    double s;
    for (size_t i = 0; i < x.size(); ++i)
    {
        s = 0.;
        for (size_t j = 0; j < coeffs.size(); ++j)
        {
            s += coeffs[j] * pow(x[i], j);
        }
        r.push_back(s);
    }
    return r;
}

vector<double> evalState(const vector<double> &coeffs, const double t)
{
    vector<double> s(coeffs.begin(), coeffs.begin() + 6);
    vector<double> d(coeffs.begin() + 6, coeffs.begin() + 12);

    auto s1 = differntiate(s);
    auto s2 = differntiate(s1);

    auto d1 = differntiate(d);
    auto d2 = differntiate(d1);

    return {polyval(s, t), polyval(s1, t), polyval(s2, t),
            polyval(d, t), polyval(d1, t), polyval(d2, t)};
}

vector<double> differntiate(const vector<double> &coeffs)
{
    if (coeffs.size() > 0)
    {
        vector<double> r;
        for (size_t i = 1; i < coeffs.size(); ++i)
        {
            r.push_back(coeffs[i] * i);
        }
        return r;
    }
    else
    {
        cerr << "Coeffs(vector) must have at least one element.\n";
    }
}

double nearest_approach_to_any_vehicle(const vector<double> &traj,
                                       const vector<Vehicle> &vehicles)
{
    double closest = 1e9, d;
    for (size_t i = 0; i < vehicles.size(); ++i)
    {
        d = nearest_approach(traj, vehicles[i]);
        if (d < closest)
        {
            closest = d;
        }
    }
    return closest;
}

double nearest_approach(const vector<double> &traj, const Vehicle &vehicle)
{
    double closest = 1e9;
    vector<double> s(traj.begin(), traj.begin() + 6);
    vector<double> d(traj.begin() + 6, traj.begin() + 12);
    // double T = traj[12];
    double target_s, target_d, dist;
    vector<double> t_vec(101);
    for (int i = 0; i < 101; ++i)
    {
        t_vec[i] = float(i) / 100.0 * HORIZON;
    }
    auto cur_s = polyval(s, t_vec);
    auto cur_d = polyval(d, t_vec);
    double dx, dy;
    for (int i = 0; i < 101; ++i)
    {
        target_s = vehicle.s[i];
        target_d = vehicle.d[i];
        dx = cur_s[i] - target_s;
        dy = cur_d[i] - target_d;
        dist = dx * dx + dy * dy;
        if (dist < closest)
        {
            closest = dist;
        }
    }
    return sqrt(closest);
}

int getLane(double d) { return d / 4; }

void printState(const vector<double> &x)
{
    cout << "state: ";
    for (int i = 0; i < 6; ++i)
    {
        cout << x[i] << ", ";
    }
    cout << "\n";
}

void printVec(const vector<double> &x)
{
    for (int i = 0; i < x.size(); ++i)
    {
        cout << x[i] << ", ";
    }
    cout << "\n";
}

void toVehicleFrame(vector<double> &x_v, vector<double> &y_v,
                    const vector<double> &x_w, const vector<double> &y_w,
                    double x_ref, double y_ref, double yaw_ref)
{
    size_t L = x_w.size();
    x_v.resize(L);
    y_v.resize(L);

    double tx, ty;
    for (size_t i = 0; i < L; ++i)
    {
        tx = x_w[i] - x_ref;
        ty = y_w[i] - y_ref;
        x_v[i] = cos(yaw_ref) * tx + sin(yaw_ref) * ty;
        y_v[i] = -sin(yaw_ref) * tx + cos(yaw_ref) * ty;
    }
}

void toWorldFrame(vector<double> &x_w, vector<double> &y_w,
                  const vector<double> &x_v, const vector<double> &y_v,
                  double x_ref, double y_ref, double yaw_ref)
{
    size_t L = x_v.size();
    x_w.resize(L);
    y_w.resize(L);

    for (size_t i = 0; i < L; ++i)
    {
        x_w[i] = cos(yaw_ref) * x_v[i] - sin(yaw_ref) * y_v[i] + x_ref;
        y_w[i] = sin(yaw_ref) * x_v[i] + cos(yaw_ref) * y_v[i] + y_ref;
    }
}

bool compareVec(TwoVect &a, TwoVect &b) { return a.v1 < b.v1; }

void sortVecs(vector<double> &v1, vector<double> &v2)
{
    auto a = v1;
    auto b = v2;
    v1.clear();
    v2.clear();
    v1.push_back(a[0]);
    v2.push_back(b[0]);

    for (size_t i = 1; i < a.size(); ++i)
    {
        if (a[i] > v1[v1.size() - 1])
        {
            v1.push_back(a[i]);
            v2.push_back(b[i]);
        }
    }
}

void sortVecs2(vector<double> &v1, vector<double> &v2)
{
    vector<TwoVect> vecS;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        vecS.push_back(TwoVect(v1[i], v2[i]));
    }
    sort(vecS.begin(), vecS.end(), compareVec);
    for (size_t i = 0; i < v1.size(); ++i)
    {
        v1[i] = vecS[i].v1;
        v2[i] = vecS[i].v2;
    }
}

double meanVecAbs(const vector<double> &x)
{
    double y = 0;
    for (auto it = x.begin(); it != x.end(); ++it)
    {
        y += abs(*it);
    }
    return y / x.size();
}

double maxVelocity(const vector<double> &vx, const vector<double> &vy)
{
    double mv = 0;
    double v;
    for (size_t i = 0; i < vx.size(); ++i)
    {
        v = vx[i] * vx[i] + vy[i] * vy[i];
        if (v > mv)
            mv = v;
    }
    return sqrt(mv);
}

double min2(double x1, double x2) { return (x1 > x2 ? x2 : x1); }