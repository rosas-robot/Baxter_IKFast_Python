/* 
 * File:   myIKFastwrap.cpp
 * Author: Anirban Sinha
 * Email: anirban.sinha.jgec@gmail.com
 * Created on October 7th, 2018, 7:19PM
 */

#include <cstdlib>

using namespace std;
#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 61
#include "baxter_left_arm_ikfast_solver.cpp" /*use right_arm solver if working with right arm*/
#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#include <Python.h>
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>
#define NPY_NO_DEPRICATED_API NPY_1_7_API_VERSION

#define IKREAL_TYPE IkReal // for IKFast 56,61

extern "C"{
    // Function wrapping Fibonacci function in C++
    static PyObject* compIKs_wrap(PyObject* /*self*/, PyObject* args)
    {
        double freeParamVal;
	int numListElement;
	PyObject* ee_pose;
	PyArrayObject* pyArrSol;
        if (!PyArg_ParseTuple(args, "dO!", &freeParamVal, &PyList_Type, &ee_pose)) {
            printf("Error parsing input argument(s)\n");
            return NULL;
        }
	numListElement = PyList_Size(ee_pose);
	if (numListElement < 12) {
	    Py_INCREF(Py_None);
	    printf("There is no 12 elements given in the ee_pose list\n");
	    return Py_None;
	}
        IkSolutionList<IkReal> solutions;
        std::vector<IkReal> vfree(GetNumFreeParameters());
        IkReal eerot[9],eetrans[3];
        bool printSol = true;
        /*Create a vector to hold all solutions*/
        std::vector<double> solVec;

	int eetrans_idx = 0;
	int eerot_idx = 0;
	for(int i = 0; i < numListElement; i++) {
	    if (i == 3 || i == 7 || i == 11) {
            	eetrans[eetrans_idx] = PyFloat_AsDouble(PyList_GetItem(ee_pose, i));
		eetrans_idx += 1;
	     }
	    else {
		eerot[eerot_idx] = PyFloat_AsDouble(PyList_GetItem(ee_pose, i));
		eerot_idx += 1;
	    }
	}	

        for(std::size_t i = 0; i < vfree.size(); ++i)
            vfree[i] = freeParamVal;

	// call ComputeIk method
	ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
	double* solArr = new double[num_of_solutions*GetNumJoints()];
	int count_idx = 0;

	std::vector<double> solvalues(GetNumJoints());
        for (std::size_t i =0; i < num_of_solutions; i++) {
     	    const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
            int this_sol_free_params = (int)sol.GetFree().size();
	    printf("sol%d (free=%d): ", (int)i, this_sol_free_params);
	    std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
	    sol.GetSolution(&solvalues[0], vsolfree.size()>0?&vsolfree[0]:NULL);
	    for(std::size_t j = 0; j < solvalues.size(); ++j)
	    {
		*(solArr + count_idx) = solvalues[j];
		count_idx++;
		printf("%.15f, ", solvalues[j]);
            }
	    printf("\n");
	}
	//printf("count_idx: %d", count_idx);
	// copy data to PyArrayObject
	npy_intp dims[2] = {num_of_solutions, GetNumJoints()};
	pyArrSol = (PyArrayObject*) PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, solArr);

	// follwoing flag moves ownership of the allocated momory on stack to NPY_ARRAY
	// and frees the memory when memory becomes out of scope
	PyArray_ENABLEFLAGS(pyArrSol, NPY_ARRAY_OWNDATA);
	std::cout << "Solution is prepared for python side\n" << std::endl;
	return Py_BuildValue("O", pyArrSol);
        //return Py_BuildValue("i", num_of_solutions);
    }

   // Wrapper function for forward kinematics
   static PyObject* compFK_wrap(PyObject* /*self*/, PyObject* args) {
	PyObject* jointAngles;
	PyArrayObject* pyArrFk;
	npy_intp dims[2] = {3, 4};

	IKREAL_TYPE eerot[9], eetrans[3];
   	IKREAL_TYPE joints[GetNumJoints()];

	
	int total_elements = 12;
	double* fkArr = new double[total_elements];
	int count_idx = 0;
	int trans_idx = 0;
	int rot_idx = 0;

	if (!PyArg_ParseTuple(args, "O!", &PyList_Type, &jointAngles)) {
            printf("Error parsing input argument(s)\n");
            return NULL;
        }
	int numListElement = PyList_Size(jointAngles);
	if (numListElement < 7) {
	    Py_INCREF(Py_None);
	    printf("There is no 7 elements given in the joint angles list\n");
	    return Py_None;
	}

	for (int i = 0; i < numListElement; i++) {
        	joints[i] = PyFloat_AsDouble(PyList_GetItem(jointAngles, i));;
        }

	// Call ComputeFk method of IKFast module
        ComputeFk(joints, eetrans, eerot);
	printf(" Translation: x: %f  y: %f  z: %f  \n", eetrans[0], eetrans[1], eetrans[2]);
	printf("\n");
	printf("    Rotation       %f   %f   %f \n", eerot[0], eerot[1], eerot[2] );
        printf("      Matrix       %f   %f   %f \n", eerot[3], eerot[4], eerot[5] );
	printf("                   %f   %f   %f \n", eerot[6], eerot[7], eerot[8] );
	
	// machinary to return FK to python side
	for (unsigned int j=0; j < total_elements; j++) {
	    if (j == 3 || j == 7 || j == 11) {
	   	 *(fkArr + count_idx) = eetrans[trans_idx];
	   	 trans_idx++;
            }
	    else {
	   	 *(fkArr + count_idx) = eerot[rot_idx];
	   	 rot_idx++;
	    }
	    count_idx++;
	}
	pyArrFk = (PyArrayObject*) PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, fkArr);

	// follwoing flag moves ownership of the allocated momory on stack to NPY_ARRAY
	// and frees the memory when memory becomes out of scope
	PyArray_ENABLEFLAGS(pyArrFk, NPY_ARRAY_OWNDATA);
	
	return Py_BuildValue("O", pyArrFk);
    }

   // An array specifying exactly which methods are wrappers
   static PyMethodDef CppMethods[] = {
	{"compIKs", compIKs_wrap, METH_VARARGS, "Computes all IKs of Baxter left/right arm"},
	{"compFK", compFK_wrap, METH_VARARGS, "Computes FK of Baxter left/right arm"},
        {NULL, NULL, 0, NULL}
   };
  // Define module information
  static struct PyModuleDef ikModule = {
	PyModuleDef_HEAD_INIT,
    "ikModule",
    "inverse_kinematics Module",
    -1,
    CppMethods
  };
  // Init function for module
  PyMODINIT_FUNC PyInit_ikModule(void) {
	import_array();
  	return PyModule_Create(&ikModule);
  }
}

