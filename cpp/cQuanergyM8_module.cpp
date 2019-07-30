
#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <math.h>
#include <arpa/inet.h>

#include "LOOKUP_COSINE.h"
#include "LOOKUP_SINE.h"
#include "Quanergy_Packet_Structures.h"

const double M8_VERTICAL_ANGLES[] = {
	-0.318505,
	-0.2692,
	-0.218009,
	-0.165195,
	-0.111003,
	-0.0557982,
	0.f,
	0.0557982
};


const float M8_vert_sin[] = {
	(float)sin(M8_VERTICAL_ANGLES[0]),
	(float)sin(M8_VERTICAL_ANGLES[1]),
	(float)sin(M8_VERTICAL_ANGLES[2]),
	(float)sin(M8_VERTICAL_ANGLES[3]),
	(float)sin(M8_VERTICAL_ANGLES[4]),
	(float)sin(M8_VERTICAL_ANGLES[5]),
	(float)sin(M8_VERTICAL_ANGLES[6]),
	(float)sin(M8_VERTICAL_ANGLES[7])
};

const float M8_vert_cos[] = {
	(float)cos(M8_VERTICAL_ANGLES[0]),
	(float)cos(M8_VERTICAL_ANGLES[1]),
	(float)cos(M8_VERTICAL_ANGLES[2]),
	(float)cos(M8_VERTICAL_ANGLES[3]),
	(float)cos(M8_VERTICAL_ANGLES[4]),
	(float)cos(M8_VERTICAL_ANGLES[5]),
	(float)cos(M8_VERTICAL_ANGLES[6]),
	(float)cos(M8_VERTICAL_ANGLES[7])
};



static PyObject *QuanergyM8_Error;

PyObject* parse_firing_data(PyObject *self, PyObject *args) {

	const char* buffer;
	int buffer_len;
	PyArrayObject *pointcloud;
	PyArrayObject *intensities;
	uint32_t start_idx;

	if (!PyArg_ParseTuple(args, "s#OOI", &buffer, &buffer_len, &pointcloud,
		&intensities, &start_idx)) {
		return NULL;
	}

	if (buffer_len != 132) {
		PyErr_SetString(QuanergyM8_Error, "buffer must be 132 bytes");
		return NULL;
	}

	if (!PyArray_Check(pointcloud)) {
		PyErr_SetString(QuanergyM8_Error, "pointcloud must be: np.array((N, 3), np.float32)");
		return NULL;
	}

	if (PyArray_NDIM(pointcloud) != 2) {
		PyErr_SetString(QuanergyM8_Error, "pointcloud must be: np.array((N, 3), np.float32)");
		return NULL;
	}
	npy_intp *pcShape = PyArray_SHAPE(pointcloud);
	if (pcShape[1] != 3) {
		PyErr_SetString(QuanergyM8_Error, "pointcloud must be: np.array((N, 3), np.float32)");
		return NULL;
	}

	PyArray_Descr *dtype = PyArray_DTYPE(pointcloud);
	if (dtype->type != 'f') {
		PyErr_SetString(QuanergyM8_Error, "pointcloud must be: np.array((N, 3), np.float32)");
		return NULL;
	}

	if (!PyArray_Check(intensities)) {
		PyErr_SetString(QuanergyM8_Error, "intensities must be an Nx1 array, uint8");
		return NULL;
	}

	//printf("buffer: %s\n", buffer);
	//printf("buffer_len is: %d\n", buffer_len);
	//printf("start_idx is: %d\n", start_idx);

	float *pcPtr = (float *)PyArray_DATA(pointcloud);
	uint8_t *intenPtr = (uint8_t *)PyArray_DATA(intensities);

	//printf("  a value: %d\n", intenPtr[4]);
	//printf("in pointcloud: %f\n", pcPtr[4*3 +1]);

	struct M8FiringData *n_firing_data = (struct M8FiringData *) buffer;
	uint16_t angle_10400 = ntohs(n_firing_data->position);
	float rotCos = M8_rot_cosine[angle_10400];
	float rotSin = M8_rot_sine[angle_10400];

	int count = 0;
	int offset = start_idx * 3;
	int returnNum = 0;
	for (int laserNum = 0; laserNum<M8_NUM_LASERS; laserNum++) {
		float vCos = M8_vert_cos[laserNum];
		float vSin = M8_vert_sin[laserNum];

		uint32_t dist = ntohl(n_firing_data->returns_distances[returnNum][laserNum]);
		if (dist > 0) {
			float real_distance = dist * 1e-5;
			pcPtr[offset    ] = real_distance * vCos * rotCos;  // x, in meters
			pcPtr[offset + 1] = real_distance * vCos * rotSin;  // y, in meters
			pcPtr[offset + 2] = real_distance * vSin;           // z, in meters
			intenPtr[offset] = n_firing_data->returns_intensities[returnNum][laserNum];
			offset += 3;
			count++;
		}
	}

	return PyLong_FromLong(count);
}

PyDoc_STRVAR (
	parse_firing_data__doc__,
	"parse_firing_data(buffer, pointcloud, intensities, start_idx)\n"
	"\n"
	"Parameters\n"
	"----------\n"
	"buffer : 132 bytes\n"
	"pointcloud : ndarray((n, 3), np.float32)\n"
	"intensities : ndarray((n), np.uint8)\n"
	"start_idx : int\n"
	"\n"
	"Returns\n"
	"-------\n"
	"out : number_of_new_points\n"
	"\n"
	"Parse an M8 Firing Data sub-packet (132 bytes).  3-D points will be written\n"
	"into \"pointcloud\" starting at position \"start_idx\".  Intensities will be\n"
	"written into \"intensities\" starting at position \"start_idx\".  Returns the\n"
	"number of new points added.\n"
);

static PyMethodDef QuanergyM8_Methods[] = {
	{"parse_firing_data", parse_firing_data, METH_VARARGS, parse_firing_data__doc__},
	{NULL, NULL, 0, NULL}
};

static struct PyModuleDef cQuanergyM8_module = {
	PyModuleDef_HEAD_INIT,
	"cQuanergyM8",
	NULL,  // TODO:  put docs here
	-1,
	QuanergyM8_Methods
};


PyMODINIT_FUNC PyInit_cQuanergyM8(void) {
	PyObject *m;

	m = PyModule_Create(&cQuanergyM8_module);
	if (m == NULL) {
		return NULL;
	}

	import_array();

	QuanergyM8_Error = PyErr_NewException("quanergyM8.error", NULL, NULL);
	Py_INCREF(QuanergyM8_Error);
	PyModule_AddObject(m, "error", QuanergyM8_Error);

	return m;
}

