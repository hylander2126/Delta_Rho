#include <math.h>

typedef struct {
	float x;
	float y;
} Vector2D;


Vector2D ALTrotateVector2D(Vector2D v, float angle) {
	Vector2D rotatedVec;
	rotatedVec.x = v.x * cos(angle) - v.y * sin(angle);
	rotatedVec.y = v.x * sin(angle) + v.y * cos(angle);
	return rotatedVec;
}

int main() {
	Vector2D vec = {1.0, 2.0};
	Vector2D vec_along_l3 = ALTrotateVector2D(vec, 3.1416 / 4);
	return 0;
}
