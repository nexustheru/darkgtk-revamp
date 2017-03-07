#include <ode/ode.h>
#pragma comment ( lib, "ODE.lib" )

// general
void		ODE_Start								( void );
void		ODE_Start								( int iSpace );
void		ODE_Update								( void );
void		ODE_Update								( float fODEStep );
void		ODE_End									( void );

// world functions
void		ODE_SetWorldGravity						( float fX, float fY, float fZ );
void		ODE_SetWorldERP							( float fValue );
void		ODE_SetWorldCFM							( float fValue );
void		ODE_SetWorldStep						( float fStep );
void		ODE_SetWorldQuickStepNumIterations		( int iValue );
void		ODE_SetStepMode							( int iMode );

float		ODE_GetWorldGravityX					( void );
float		ODE_GetWorldGravityY					( void );
float		ODE_GetWorldGravityZ					( void );
float		ODE_GetWorldERP							( void );
float		ODE_GetWorldCFM							( void );
float		ODE_GetWorldStep						( void );
int			ODE_GetWorldQuickStepNumIterations		( void );

// disable functions
void		ODE_SetAutoDisableFlag					( int iFlag );
void		ODE_SetAutoDisableLinearThreshold		( float fThreshold );
void		ODE_SetAutoDisableAngularThreshold		( float fThreshold );
void		ODE_SetAutoDisableSteps					( int iSteps );
void		ODE_SetAutoDisableTime					( float fTime );
int			ODE_GetAutoDisableFlag					( void );
float		ODE_GetAutoDisableLinearThreshold		( void );
float		ODE_GetAutoDisableAngularThreshold		( void );
int			ODE_GetAutoDisableSteps					( void );
float		ODE_GetAutoDisableTime					( void );

// contact parameters
void		ODE_SetWorldContactSurfaceLayer			( float fDepth );
float		ODE_GetWorldContactSurfaceLayer			( void );

// rigid body functions
void		ODE_SetBodyPosition						( int iID, float fX, float fY, float fZ );
void		ODE_SetBodyRotation						( int iID, float fX, float fY, float fZ );
void		ODE_SetBodyLinearVelocity				( int iID, float fX, float fY, float fZ );
void		ODE_SetBodyAngularVelocity				( int iID, float fX, float fY, float fZ );
float		ODE_GetBodyPositionX					( int iID );
float		ODE_GetBodyPositionY					( int iID );
float		ODE_GetBodyPositionZ					( int iID );
float		ODE_GetBodyRotationX					( int iID );
float		ODE_GetBodyRotationY					( int iID );
float		ODE_GetBodyRotationZ					( int iID );
DWORD		ODE_GetBodyLinearVelocityX				( int iID );
DWORD		ODE_GetBodyLinearVelocityY				( int iID );
DWORD		ODE_GetBodyLinearVelocityZ				( int iID );
DWORD		ODE_GetBodyAngularVelocityX				( int iID );
DWORD		ODE_GetBodyAngularVelocityY				( int iID );
DWORD		ODE_GetBodyAngularVelocityZ				( int iID );

void		ODE_AddBodyForce						( int iObject, float fX, float fY, float fZ, float fPX, float fPY, float fPZ );
DWORD		ODE_GetObjectAContact					( void );
DWORD		ODE_GetObjectBContact					( void );
void		ODE_SetBodyResponse						( int iID, int iResponseMode );
DWORD		ODE_GetBodyHeight						( int iID );
DWORD		ODE_GetBodyAdjustmentX					( int iID );
DWORD		ODE_GetBodyAdjustmentY					( int iID );
DWORD		ODE_GetBodyAdjustmentZ					( int iID );

// car functions
void		ODE_CreateCar							( int iCar );
void		ODE_SetCarPosition						( int iCar, float fX, float fY, float fZ );
void		ODE_SetCarMass							( int iCar, float fMass );
void		ODE_SetCarMesh							( int iCar, int iObject, int iMode );
void		ODE_SetCarWheel							( int iCar, int iWheel, int iObject, float fRadius );
void		ODE_SetCarWheelMass						( int iCar, int iWheel, float fMass );
void		ODE_SetCarWheelRotation					( int iCar, int iWheel, float fX, float fY, float fZ );
void		ODE_SetCarWheelPosition					( int iCar, int iWheel, float fX, float fY, float fZ );
void		ODE_SetCarDefaultJoints					( int iCar );
void		ODE_UpdateCar							( int iCar );

// creation and destruction
void		ODE_CreateStaticBox						( int iObject, float fXSize, float fYSize, float fZSize );
void		ODE_CreateStaticSphere					( int iObject );
void		ODE_CreateStaticBox						( int iObject );
void		ODE_CreateStaticTriangleMesh			( int iObject );
void		ODE_CreateDynamicSphere					( int iObject );
void		ODE_CreateDynamicBox					( int iObject );
void		ODE_CreateDynamicBox					( int iObject, float fXSize, float fYSize, float fZSize );
void		ODE_CreateDynamicTriangleMesh			( int iObject );
void		ODE_CreateDynamicCylinder				( int iObject );
void		ODE_CreateStaticUniverse				( void );
void		ODE_DestroyObject						( int iObject );

// body properties
void		ODE_SetLinearVelocity					( int iObject, float fX, float fY, float fZ );
void		ODE_SetAngularVelocity					( int iObject, float fX, float fY, float fZ );
void		ODE_SetGravity							( int iObject, int iMode );
void		ODE_SetBodyRotation						( int iObject, float fX, float fY, float fZ );
void		ODE_SetBodyMass							( int iObject, float fMass );
void		ODE_BodyEnable							( int iID );
void		ODE_BodyDisable							( int iID );
int			ODE_GetBodyEnabledFlag					( int iID );

// surface properties
void		ODE_SetSurfaceMode						( int iID, int iMode, int iState );
void		ODE_SetSurfaceModeContactMu2			( int iID, int iState );
void		ODE_SetSurfaceModeContactFDir1			( int iID, int iState );
void		ODE_SetSurfaceModeContactBounce			( int iID, int iState );
void		ODE_SetSurfaceModeContactSoftERP		( int iID, int iState );
void		ODE_SetSurfaceModeContactSoftCFM		( int iID, int iState );
void		ODE_SetSurfaceModeContactMotion1		( int iID, int iState );
void		ODE_SetSurfaceModeContactMotion2		( int iID, int iState );
void		ODE_SetSurfaceModeContactSlip1			( int iID, int iState );
void		ODE_SetSurfaceModeContactSlip2			( int iID, int iState );
void		ODE_SetSurfaceModeContactApprox0		( int iID, int iState );
void		ODE_SetSurfaceModeContactApprox11		( int iID, int iState );
void		ODE_SetSurfaceModeContactApprox12		( int iID, int iState );
void		ODE_SetSurfaceModeContactApprox1		( int iID, int iState );

void		ODE_SetContact							( int iID, int iMode, float fValue );
void		ODE_SetContactMu2						( int iID, float fValue );
void		ODE_SetContactFDir1						( int iID, float fValue );
void		ODE_SetContactBounce					( int iID, float fValue );
void		ODE_SetContactBounceVelocity			( int iID, float fValue );
void		ODE_SetContactSoftERP					( int iID, float fValue );
void		ODE_SetContactSoftCFM					( int iID, float fValue );
void		ODE_SetContactMotion1					( int iID, float fValue );
void		ODE_SetContactMotion2					( int iID, float fValue );
void		ODE_SetContactSlip1						( int iID, float fValue );
void		ODE_SetContactSlip2						( int iID, float fValue );

int			ODE_CollisionMessageExists				( void );
void		ODE_CollisionGetMessage					( void );
int			ODE_GetObjectA							( void );
int			ODE_GetObjectB							( void );
DWORD		ODE_GetObjectAVelocityX					( void );
DWORD		ODE_GetObjectAVelocityY					( void );
DWORD		ODE_GetObjectAVelocityZ					( void );
DWORD		ODE_GetObjectAAngularVelocityX			( void );
DWORD		ODE_GetObjectAAngularVelocityY			( void );
DWORD		ODE_GetObjectAAngularVelocityZ			( void );
DWORD		ODE_GetObjectBVelocityX					( void );
DWORD		ODE_GetObjectBVelocityY					( void );
DWORD		ODE_GetObjectBVelocityZ					( void );
DWORD		ODE_GetObjectBAngularVelocityX			( void );
DWORD		ODE_GetObjectBAngularVelocityY			( void );
DWORD		ODE_GetObjectBAngularVelocityZ			( void );
