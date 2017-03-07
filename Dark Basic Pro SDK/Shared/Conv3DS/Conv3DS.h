//
// Conv3DS Functions Header
//

//////////////////////////////////////////////////////////////////////////////////
// INCLUDE COMMON FILES //////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#include <windows.h>


//////////////////////////////////////////////////////////////////////////////////////
// DBOFORMAT INCLUDE /////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

#include "..\\DBOFormat\\DBOBlock.h"
#include ".\..\DBOFormat\DBOFormat.h"
#include ".\..\DBOFormat\DBOFrame.h"
#include ".\..\DBOFormat\DBOMesh.h"
#include ".\..\DBOFormat\DBORawMesh.h"
#include ".\..\DBOFormat\DBOEffects.h"
#include ".\..\DBOFormat\DBOFile.h"
#include ".\..\core\globstruct.h"

#ifdef DARKSDK_COMPILE
	#undef DARKSDK
	#undef DBPRO_GLOBAL
	#define DARKSDK static
	#define DBPRO_GLOBAL static
#else
	#define DARKSDK __declspec ( dllexport )
	#define DBPRO_GLOBAL 
	#define DARKSDK_DLL 
#endif

//////////////////////////////////////////////////////////////////////////////////
// DEFINES ///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//#define DARKSDK __declspec ( dllexport )
//#define SAFE_DELETE( p )       { if ( p ) { delete ( p );       ( p ) = NULL; } }
//#define SAFE_RELEASE( p )      { if ( p ) { ( p )->Release ( ); ( p ) = NULL; } }
//#define SAFE_DELETE_ARRAY( p ) { if ( p ) { delete [ ] ( p );   ( p ) = NULL; } }

//////////////////////////////////////////////////////////////////////////////////////
// EXPORTED FUNCTIONS ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
#ifndef DARKSDK_COMPILE
extern "C"
{
	DARKSDK void	PassCoreData	( LPVOID pGlobPtr );
	DARKSDK	bool	Convert			( LPSTR pFilename, DWORD *pBlock, DWORD* pdwSize );
	DARKSDK void	Free			( LPSTR );
}
#else
	void	PassCoreData3DS	( LPVOID pGlobPtr );
	bool	Convert3DS		( LPSTR pFilename, DWORD *pBlock, DWORD* pdwSize );
	void	Free3DS			( LPSTR );
#endif
