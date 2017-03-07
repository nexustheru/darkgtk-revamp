////////////////////////////////////////////////////////////////////
// DEFINES AND INCLUDES ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

#ifdef DARKSDK_COMPILE
	#include "DarkSDK.h"
#endif

#include ".\..\core\globstruct.h"
#include ".\..\DBOFormat\DBOData.h"

//#include <stdio.h>
#include <ode/ode.h>
#include <vector>

#ifndef DARKSDK_COMPILE
	#define DARKSDK	__declspec ( dllexport )
#endif

#define WIN32_LEAN_AND_MEAN

#define SAFE_DELETE( p )		{ if ( p ) { delete ( p );       ( p ) = NULL; } }
#define SAFE_RELEASE( p )		{ if ( p ) { ( p )->Release ( ); ( p ) = NULL; } }
#define SAFE_DELETE_ARRAY( p )	{ if ( p ) { delete [ ] ( p );   ( p ) = NULL; } }

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

struct sODEObject
{
	dBodyID		body;
	dGeomID		geom;
	int			iID;
	int			iFrame;
	dMatrix3	rotation;

	float		fXSize;
	float		fYSize;
	float		fZSize;
	float		fXAngle;
	float		fYAngle;
	float		fZAngle;

	bool		bNoNeedToRecalculateShape;
	int			iResponseMode;
	int			iItemCarryGrab;
	dMatrix3	wItemCarryGrab;
	float		fActualMarginX;
	float		fActualMarginY;
	float		fActualMarginZ;
	float		fActualHeight;

	int			iSurfaceMode;
	float		fContactMU;
	float		fContactMU2;
	float		fContactBounce;
	float		fContactBounceVelocity;
	float		fContactSoftERP;
	float		fContactSoftCFM;
	float		fContactMotion1;
	float		fContactMotion2;
	float		fContactSlip1;
	float		fContactSlip2;
};

struct sODECar
{
	float		fX;
	float		fY;
	float		fZ;

	float		fMass;
	float		fWheelRadius;

	dSpaceID	space;
	dJointID	joints [ 4 ];
	dGeomID		geom   [ 5 ];
	dBodyID		bodies [ 5 ];
	int			iID    [ 5 ];
};

struct sODECollision
{
	int iObjectA;
	int iObjectB;

	float fObjectAContact;
	float fObjectAVelocityX;
	float fObjectAVelocityY;
	float fObjectAVelocityZ;
	float fObjectAAngularVelocityX;
	float fObjectAAngularVelocityY;
	float fObjectAAngularVelocityZ;

	float fObjectBContact;
	float fObjectBVelocityX;
	float fObjectBVelocityY;
	float fObjectBVelocityZ;
	float fObjectBAngularVelocityX;
	float fObjectBAngularVelocityY;
	float fObjectBAngularVelocityZ;
};

#ifdef DARKSDK_COMPILE
	extern dWorldID						g_ODEWorld;
	extern dSpaceID						g_ODESpace;
	extern dJointGroupID				g_ODEContactGroup;
	extern std::vector < sODEObject >	g_ODEObjectList;
	extern std::vector < sODEObject >	g_ODEStaticObjectList;
	extern sODECar						g_ODECarList [ 64 ];
#endif

// internal functions
static void			ODE_Callback							( void* data, dGeomID o1, dGeomID o2 );
void				ODE_AddObject							( dBodyID body, dGeomID geom, int iID, int iFrame );
void				ODE_AddObject							( sODEObject object, int iID, int iFrame );
sODEObject*			ODE_FindID								( int iID );
int					ODE_FindID								( dBodyID body );

// general
DARKSDK void		ODE_Start								( void );
DARKSDK void		ODE_Start								( int iSpace );
DARKSDK void		ODE_Update								( void );
DARKSDK void		ODE_Update								( float fODEStep );
DARKSDK void		ODE_End									( void );

// world functions
DARKSDK void		ODE_SetWorldGravity						( float fX, float fY, float fZ );
DARKSDK void		ODE_SetWorldERP							( float fValue );
DARKSDK void		ODE_SetWorldCFM							( float fValue );
DARKSDK void		ODE_SetWorldStep						( float fStep );
DARKSDK void		ODE_SetWorldQuickStepNumIterations		( int iValue );
DARKSDK void		ODE_SetStepMode							( int iMode );

DARKSDK float		ODE_GetWorldGravityX					( void );
DARKSDK float		ODE_GetWorldGravityY					( void );
DARKSDK float		ODE_GetWorldGravityZ					( void );
DARKSDK float		ODE_GetWorldERP							( void );
DARKSDK float		ODE_GetWorldCFM							( void );
DARKSDK float		ODE_GetWorldStep						( void );
DARKSDK int			ODE_GetWorldQuickStepNumIterations		( void );

// disable functions
DARKSDK void		ODE_SetAutoDisableFlag					( int iFlag );
DARKSDK void		ODE_SetAutoDisableLinearThreshold		( float fThreshold );
DARKSDK void		ODE_SetAutoDisableAngularThreshold		( float fThreshold );
DARKSDK void		ODE_SetAutoDisableSteps					( int iSteps );
DARKSDK void		ODE_SetAutoDisableTime					( float fTime );
DARKSDK int			ODE_GetAutoDisableFlag					( void );
DARKSDK float		ODE_GetAutoDisableLinearThreshold		( void );
DARKSDK float		ODE_GetAutoDisableAngularThreshold		( void );
DARKSDK int			ODE_GetAutoDisableSteps					( void );
DARKSDK float		ODE_GetAutoDisableTime					( void );

// contact parameters
DARKSDK void		ODE_SetWorldContactSurfaceLayer			( float fDepth );
DARKSDK float		ODE_GetWorldContactSurfaceLayer			( void );

// rigid body functions
DARKSDK void		ODE_SetBodyPosition						( int iID, float fX, float fY, float fZ );
DARKSDK void		ODE_SetBodyRotation						( int iID, float fX, float fY, float fZ );
DARKSDK void		ODE_SetBodyLinearVelocity				( int iID, float fX, float fY, float fZ );
DARKSDK void		ODE_SetBodyAngularVelocity				( int iID, float fX, float fY, float fZ );
DARKSDK float		ODE_GetBodyPositionX					( int iID );
DARKSDK float		ODE_GetBodyPositionY					( int iID );
DARKSDK float		ODE_GetBodyPositionZ					( int iID );
DARKSDK float		ODE_GetBodyRotationX					( int iID );
DARKSDK float		ODE_GetBodyRotationY					( int iID );
DARKSDK float		ODE_GetBodyRotationZ					( int iID );
DARKSDK DWORD		ODE_GetBodyLinearVelocityX				( int iID );
DARKSDK DWORD		ODE_GetBodyLinearVelocityY				( int iID );
DARKSDK DWORD		ODE_GetBodyLinearVelocityZ				( int iID );
DARKSDK DWORD		ODE_GetBodyAngularVelocityX				( int iID );
DARKSDK DWORD		ODE_GetBodyAngularVelocityY				( int iID );
DARKSDK DWORD		ODE_GetBodyAngularVelocityZ				( int iID );

DARKSDK void		ODE_AddBodyForce						( int iObject, float fX, float fY, float fZ, float fPX, float fPY, float fPZ );
DARKSDK DWORD		ODE_GetObjectAContact					( void );
DARKSDK DWORD		ODE_GetObjectBContact					( void );
DARKSDK void		ODE_SetBodyResponse						( int iID, int iResponseMode );
DARKSDK DWORD		ODE_GetBodyHeight						( int iID );
DARKSDK DWORD		ODE_GetBodyAdjustmentX					( int iID );
DARKSDK DWORD		ODE_GetBodyAdjustmentY					( int iID );
DARKSDK DWORD		ODE_GetBodyAdjustmentZ					( int iID );

// car functions
DARKSDK void		ODE_CreateCar							( int iCar );
DARKSDK void		ODE_SetCarPosition						( int iCar, float fX, float fY, float fZ );
DARKSDK void		ODE_SetCarMass							( int iCar, float fMass );
DARKSDK void		ODE_SetCarMesh							( int iCar, int iObject, int iMode );
DARKSDK void		ODE_SetCarWheel							( int iCar, int iWheel, int iObject, float fRadius );
DARKSDK void		ODE_SetCarWheelMass						( int iCar, int iWheel, float fMass );
DARKSDK void		ODE_SetCarWheelRotation					( int iCar, int iWheel, float fX, float fY, float fZ );
DARKSDK void		ODE_SetCarWheelPosition					( int iCar, int iWheel, float fX, float fY, float fZ );
DARKSDK void		ODE_SetCarDefaultJoints					( int iCar );
DARKSDK void		ODE_UpdateCar							( int iCar );

// creation and destruction
DARKSDK void		ODE_CreateStaticBox						( int iObject, float fXSize, float fYSize, float fZSize );
DARKSDK void		ODE_CreateStaticSphere					( int iObject );
DARKSDK void		ODE_CreateStaticBox						( int iObject );
DARKSDK void		ODE_CreateStaticTriangleMesh			( int iObject );
DARKSDK void		ODE_CreateDynamicSphere					( int iObject );
DARKSDK void		ODE_CreateDynamicBox					( int iObject );
DARKSDK void		ODE_CreateDynamicBox					( int iObject, float fXSize, float fYSize, float fZSize );
DARKSDK void		ODE_CreateDynamicTriangleMesh			( int iObject );
DARKSDK void		ODE_CreateDynamicCylinder				( int iObject );
DARKSDK void		ODE_CreateStaticUniverse				( void );
DARKSDK void		ODE_DestroyObject						( int iObject );

// body properties
DARKSDK void		ODE_SetLinearVelocity					( int iObject, float fX, float fY, float fZ );
DARKSDK void		ODE_SetAngularVelocity					( int iObject, float fX, float fY, float fZ );
DARKSDK void		ODE_SetGravity							( int iObject, int iMode );
DARKSDK void		ODE_SetBodyRotation						( int iObject, float fX, float fY, float fZ );
DARKSDK void		ODE_SetBodyMass							( int iObject, float fMass );
DARKSDK void		ODE_BodyEnable							( int iID );
DARKSDK void		ODE_BodyDisable							( int iID );
DARKSDK int			ODE_GetBodyEnabledFlag					( int iID );

// surface properties
DARKSDK void		ODE_SetSurfaceMode						( int iID, int iMode, int iState );
DARKSDK void		ODE_SetSurfaceModeContactMu2			( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactFDir1			( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactBounce			( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactSoftERP		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactSoftCFM		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactMotion1		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactMotion2		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactSlip1			( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactSlip2			( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactApprox0		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactApprox11		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactApprox12		( int iID, int iState );
DARKSDK void		ODE_SetSurfaceModeContactApprox1		( int iID, int iState );

DARKSDK void		ODE_SetContact							( int iID, int iMode, float fValue );
DARKSDK void		ODE_SetContactMu2						( int iID, float fValue );
DARKSDK void		ODE_SetContactFDir1						( int iID, float fValue );
DARKSDK void		ODE_SetContactBounce					( int iID, float fValue );
DARKSDK void		ODE_SetContactBounceVelocity			( int iID, float fValue );
DARKSDK void		ODE_SetContactSoftERP					( int iID, float fValue );
DARKSDK void		ODE_SetContactSoftCFM					( int iID, float fValue );
DARKSDK void		ODE_SetContactMotion1					( int iID, float fValue );
DARKSDK void		ODE_SetContactMotion2					( int iID, float fValue );
DARKSDK void		ODE_SetContactSlip1						( int iID, float fValue );
DARKSDK void		ODE_SetContactSlip2						( int iID, float fValue );

DARKSDK int			ODE_CollisionMessageExists				( void );
DARKSDK void		ODE_CollisionGetMessage					( void );
DARKSDK int			ODE_GetObjectA							( void );
DARKSDK int			ODE_GetObjectB							( void );
DARKSDK DWORD		ODE_GetObjectAVelocityX					( void );
DARKSDK DWORD		ODE_GetObjectAVelocityY					( void );
DARKSDK DWORD		ODE_GetObjectAVelocityZ					( void );
DARKSDK DWORD		ODE_GetObjectAAngularVelocityX			( void );
DARKSDK DWORD		ODE_GetObjectAAngularVelocityY			( void );
DARKSDK DWORD		ODE_GetObjectAAngularVelocityZ			( void );
DARKSDK DWORD		ODE_GetObjectBVelocityX					( void );
DARKSDK DWORD		ODE_GetObjectBVelocityY					( void );
DARKSDK DWORD		ODE_GetObjectBVelocityZ					( void );
DARKSDK DWORD		ODE_GetObjectBAngularVelocityX			( void );
DARKSDK DWORD		ODE_GetObjectBAngularVelocityY			( void );
DARKSDK DWORD		ODE_GetObjectBAngularVelocityZ			( void );
