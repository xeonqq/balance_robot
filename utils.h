#define Rad2Deg 57.29577951308232
#define Deg2Rad 0.017453292519943295

#define sqr(x) ((x)*(x))
template<class TYPE>
TYPE normalizeAngle(TYPE angle) {
    const TYPE pi2 = M_PI * 2;
    while (angle > M_PI) angle -= pi2;
    while (angle < -M_PI) angle += pi2;

    return angle;
}











