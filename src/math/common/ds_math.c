#include <float.h>
#include "ds_matrix.c"
#include "geometry.c"
#include "transform.c"
#include "ds_random.c"

static void ds_FloatStaticAssert(void)
{
	ds_StaticAssert(F32_EPSILON == FLT_EPSILON, "Our machine epsilon does not equate to FLT_EPSILON");
}

