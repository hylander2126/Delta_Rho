//================================================================================================
//                                         Vector Algebra
//================================================================================================

void dot_product_3D (float *V1, float *V2, float *R)
{

	*R = V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2];

}

//________________________________________________________________________________________________
void cross_product_3D (float *V1, float *V2, float *R)
{

	R[0] = V1[1]*V2[2] - V1[2]*V2[1];
	R[1] = V1[2]*V2[0] - V1[0]*V2[2];
	R[2] = V1[0]*V2[1] - V1[1]*V2[0];

}

//________________________________________________________________________________________________
void vector_normalize_3D (float *V){

	float S;

	S = sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);

	V[0] = V[0]/S;
	V[1] = V[1]/S;
	V[2] = V[2]/S;

}