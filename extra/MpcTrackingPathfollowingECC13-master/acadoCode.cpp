/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 6){ 
      mexErrMsgTxt("This problem expects 6 right hand side argument(s) since you have defined 6 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState t;
    DifferentialState x1;
    DifferentialState x2;
    DifferentialState x3;
    DifferentialState L;
    Control u1;
    Control u2;
    double *mexinput0_temp = NULL; 
    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || !(mxGetM(prhs[0])==1 && mxGetN(prhs[0])==1) ) { 
      mexErrMsgTxt("Input 0 must be a noncomplex scalar double.");
    } 
    mexinput0_temp = mxGetPr(prhs[0]); 
    double mexinput0 = *mexinput0_temp; 

    double *mexinput1_temp = NULL; 
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || !(mxGetM(prhs[1])==1 && mxGetN(prhs[1])==1) ) { 
      mexErrMsgTxt("Input 1 must be a noncomplex scalar double.");
    } 
    mexinput1_temp = mxGetPr(prhs[1]); 
    double mexinput1 = *mexinput1_temp; 

    double *mexinput2_temp = NULL; 
    if( !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || !(mxGetM(prhs[2])==1 && mxGetN(prhs[2])==1) ) { 
      mexErrMsgTxt("Input 2 must be a noncomplex scalar double.");
    } 
    mexinput2_temp = mxGetPr(prhs[2]); 
    double mexinput2 = *mexinput2_temp; 

    double *mexinput3_temp = NULL; 
    if( !mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || !(mxGetM(prhs[3])==1 && mxGetN(prhs[3])==1) ) { 
      mexErrMsgTxt("Input 3 must be a noncomplex scalar double.");
    } 
    mexinput3_temp = mxGetPr(prhs[3]); 
    double mexinput3 = *mexinput3_temp; 

    double *mexinput4_temp = NULL; 
    if( !mxIsDouble(prhs[4]) || mxIsComplex(prhs[4]) ) { 
      mexErrMsgTxt("Input 4 must be a noncomplex double vector of dimension XxY.");
    } 
    mexinput4_temp = mxGetPr(prhs[4]); 
    DMatrix mexinput4(mxGetM(prhs[4]), mxGetN(prhs[4]));
    for( int i=0; i<mxGetN(prhs[4]); ++i ){ 
        for( int j=0; j<mxGetM(prhs[4]); ++j ){ 
           mexinput4(j,i) = mexinput4_temp[i*mxGetM(prhs[4]) + j];
        } 
    } 

    double *mexinput5_temp = NULL; 
    if( !mxIsDouble(prhs[5]) || mxIsComplex(prhs[5]) ) { 
      mexErrMsgTxt("Input 5 must be a noncomplex double vector of dimension XxY.");
    } 
    mexinput5_temp = mxGetPr(prhs[5]); 
    DMatrix mexinput5(mxGetM(prhs[5]), mxGetN(prhs[5]));
    for( int i=0; i<mxGetN(prhs[5]); ++i ){ 
        for( int j=0; j<mxGetM(prhs[5]); ++j ){ 
           mexinput5(j,i) = mexinput5_temp[i*mxGetM(prhs[5]) + j];
        } 
    } 

    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(t) == 1.000000E+00;
    acadodata_f1 << dot(x1) == cos(x3)*u1;
    acadodata_f1 << dot(x2) == sin(x3)*u1;
    acadodata_f1 << dot(x3) == u2;
    acadodata_f1 << dot(L) == (((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*1.000000E+01*cos(x3)+(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*1.000000E+01*sin(x3)+2.000000E+00)*((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*cos(x3)+(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*sin(x3)+2.000000E-01)+((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*1.000000E+01*sin(x3)-(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*1.000000E+01*cos(x3))*((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*sin(x3)-(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*cos(x3))+pow(((-1.000000E+01*cos(1/2.000000E+01*t)+x1)/1.000000E+01*cos(x3)+(-1.000000E+01*sin(1/2.000000E+01*t)+x2)/1.000000E+01*sin(x3)-1/2.000000E+00*cos(1/2.000000E+01*t)*sin(x3)+1/2.000000E+00*cos(x3)*sin(1/2.000000E+01*t)+2.000000E-02+u1),2.000000E+00)+pow(((-1.000000E+01*cos(1/2.000000E+01*t)+x1)/2.000000E+00*sin(x3)-(-1.000000E+01*sin(1/2.000000E+01*t)+x2)/2.000000E+00*cos(x3)+1/2.000000E+00*5.000000E+00*cos(1/2.000000E+01*t)*cos(x3)+1/2.000000E+00*5.000000E+00*sin(1/2.000000E+01*t)*sin(x3)-u2),2.000000E+00));

    OCP ocp1(0, 0.5, 5);
    ocp1.minimizeMayerTerm((5.000000E+01*pow(((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*cos(x3)+(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*sin(x3)+2.000000E-01),2.000000E+00)+5.000000E+01*pow(((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*sin(x3)-(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*cos(x3)),2.000000E+00)+L));
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, t == mexinput0);
    ocp1.subjectTo(AT_START, x1 == mexinput1);
    ocp1.subjectTo(AT_START, x2 == mexinput2);
    ocp1.subjectTo(AT_START, x3 == mexinput3);
    ocp1.subjectTo(AT_START, L == 0.000000E+00);
    ocp1.subjectTo((-3.000000E+00+u1) <= 0.000000E+00);
    ocp1.subjectTo((-6.283185E+00+u2) <= 0.000000E+00);
    ocp1.subjectTo((-3.000000E+00-u1) <= 0.000000E+00);
    ocp1.subjectTo((-6.283185E+00-u2) <= 0.000000E+00);
    ocp1.subjectTo(AT_END, (((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*cos(x3)+(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*sin(x3)+2.000000E-01)*((-1.000000E+01*cos(1/2.000000E+01*t)+x1)/2.000000E+00*cos(x3)+(-1.000000E+01*sin(1/2.000000E+01*t)+x2)/2.000000E+00*sin(x3)+1.000000E-01)+((-1.000000E+01*cos(1/2.000000E+01*t)+x1)*sin(x3)-(-1.000000E+01*sin(1/2.000000E+01*t)+x2)*cos(x3))*((-1.000000E+01*cos(1/2.000000E+01*t)+x1)/2.000000E+00*sin(x3)-(-1.000000E+01*sin(1/2.000000E+01*t)+x2)/2.000000E+00*cos(x3))-2.862498E+01) <= 0.000000E+00 <= 0.000000E+00);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-04 );
    algo1.set( MAX_NUM_ITERATIONS, 30 );
    algo1.initializeDifferentialStates( mexinput5 );
    algo1.initializeControls( mexinput4 );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

