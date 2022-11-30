/*--------------------------------------------------------------------------
Copyright (c) 2009, 2015, 2018-2019, 2021,The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of The Linux Foundation nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/
/*============================================================================
                            O p e n M A X   w r a p p e r s
                             O p e n  M A X   C o r e

  This module contains the implementation of the OpenMAX core.

*//*========================================================================*/

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#include <dlfcn.h>           // dynamic library
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

#include "qc_omx_core.h"
#include "omx_core_cmp.h"
#include <cutils/properties.h>

#ifndef VIDC_STUB_HAL
#include "ConfigStore.h"
#endif

extern omx_core_cb_type core[];
extern const unsigned int SIZE_OF_CORE;
static pthread_mutex_t lock_core = PTHREAD_MUTEX_INITIALIZER;
static int number_of_adec_nt_session;

extern omx_core_cb_type component[];
unsigned int num_components = 0;

#define MAX_AUDIO_NT_SESSION 2

/* ======================================================================
FUNCTION
  omx_core_load_cmp_library

DESCRIPTION
  Loads up the libary name mentioned in the argument

PARAMETERS
  None

RETURN VALUE
  Constructor for creating component instances.
========================================================================== */
static create_qc_omx_component
omx_core_load_cmp_library(char *libname, void **handle_ptr)
{
  create_qc_omx_component fn_ptr = NULL;
  if(handle_ptr)
  {
    DEBUG_PRINT("Dynamically Loading the library : %s\n",libname);
    if (!strcmp(libname, "libOmxVpp.so"))
        *handle_ptr = dlopen(libname, RTLD_NOW|RTLD_GLOBAL);
    else
        *handle_ptr = dlopen(libname, RTLD_NOW);

    if(*handle_ptr)
    {
      fn_ptr = dlsym(*handle_ptr, "get_omx_component_factory_fn");

      if(fn_ptr == NULL)
      {
        DEBUG_PRINT("Error: Library %s incompatible as QCOM OMX component loader - %s\n",
                  libname, dlerror());
        *handle_ptr = NULL;
      }
    }
    else
    {
      DEBUG_PRINT("Error: Couldn't load %s: %s\n",libname,dlerror());
    }
  }
  return fn_ptr;
}

/* ======================================================================
FUNCTION
  OMX_Init

DESCRIPTION
  This is the first function called by the application.
  There is nothing to do here since components shall be loaded
  whenever the get handle method is called.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
OMX_API OMX_ERRORTYPE OMX_APIENTRY
OMX_Init()
{
    char platform_name[PROP_VALUE_MAX] = {0};
    char version[PROP_VALUE_MAX] = {0};
    property_get("ro.board.platform", platform_name, "0");

  DEBUG_PRINT("OMXCORE API - OMX_Init \n");
  // Use below method to generate list of components actually supported
  // on any given platform. Core list is considered as superset and does
  // not determine the actual supported codecs on a particular target.
  num_components = 0;
  for (int i = 0; i < SIZE_OF_CORE; i++) {
      if (!strncmp(platform_name, "lito", 4)) {
          if (!strcmp("OMX.qcom.video.decoder.vp8", core[i].name) || !strcmp("OMX.qcom.video.encoder.vp8", core[i].name)) {
                //Bitra (SM6350) both does not support VP8 encoder and decoder hence don't add them in list
              if (property_get("vendor.media.target.version", version, "0") && ((atoi(version) == 2) || (atoi(version) == 3)))
                  continue;
          }
      }

    memcpy(&component[num_components++],
           &core[i], sizeof(omx_core_cb_type));
    }
  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  get_cmp_index

DESCRIPTION
  Obtains the  index associated with the name.

PARAMETERS
  None

RETURN VALUE
  Error None.
========================================================================== */
static int get_cmp_index(char *cmp_name)
{
  int rc = -1,i=0;
  DEBUG_PRINT("before get_cmp_index **********%d\n", rc);

  for (i = 0; i < num_components; i++)
  {
    DEBUG_PRINT("get_cmp_index: cmp_name = %s , core[i].name = %s ,count = %d \n",
                cmp_name, component[i].name, i);

    if (!strcmp(cmp_name, component[i].name))
    {
        rc = i;
        break;
    }
  }
  DEBUG_PRINT("returning index %d\n", rc);
  return rc;
}

/* ======================================================================
FUNCTION
  clear_cmp_handle

DESCRIPTION
  Clears the component handle from the component table.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
static void clear_cmp_handle(OMX_HANDLETYPE inst)
{
  unsigned i = 0,j=0;

  if(NULL == inst)
     return;

  for (i = 0; i < num_components; i++)
  {
    for(j=0; j< OMX_COMP_MAX_INST; j++)
    {
      if (inst == component[i].inst[j])
      {
        component[i].inst[j] = NULL;
        return;
      }
    }
  }
  return;
}
/* ======================================================================
FUNCTION
  is_cmp_handle_exists

DESCRIPTION
  Check if the component handle already exists or not.

PARAMETERS
  None

RETURN VALUE
  index pointer if the handle exists
  negative value otherwise
========================================================================== */
static int is_cmp_handle_exists(OMX_HANDLETYPE inst)
{
  unsigned i=0,j=0;
  int rc = -1;

  if(NULL == inst)
     return rc;

  pthread_mutex_lock(&lock_core);
  for (i = 0; i < num_components; i++)
  {
    for(j=0; j< OMX_COMP_MAX_INST; j++)
    {
      if (inst == component[i].inst[j])
      {
        rc = i;
        goto finish;
      }
    }
  }
finish:
  pthread_mutex_unlock(&lock_core);
  return rc;
}

/* ======================================================================
FUNCTION
  get_comp_handle_index

DESCRIPTION
  Gets the index to store the next handle for specified component name.

PARAMETERS
  cmp_name : Component Name

RETURN VALUE
  Index of next handle to be stored
========================================================================== */
static int get_comp_handle_index(char *cmp_name)
{
  unsigned i=0,j=0;
  int rc = -1;
  for (i = 0; i < num_components; i++)
  {
    if (!strcmp(cmp_name, component[i].name))
    {
      for(j=0; j< OMX_COMP_MAX_INST; j++)
      {
        if (NULL == component[i].inst[j])
        {
          rc = j;
          DEBUG_PRINT("free handle slot exists %d\n", rc);
          return rc;
        }
      }
      break;
    }
  }
  return rc;
}

/* ======================================================================
FUNCTION
  check_lib_unload

DESCRIPTION
  Check if any component instance is using the library

PARAMETERS
  index: Component Index in core array.

RETURN VALUE
  1: Library Unused and can be unloaded.
  0:  Library used and shouldnt be unloaded.
========================================================================== */
static int check_lib_unload(int index)
{
  unsigned i=0;
  int rc = 1;

  for(i=0; i< OMX_COMP_MAX_INST; i++)
  {
    if (component[index].inst[i])
    {
      rc = 0;
      DEBUG_PRINT("Library Used \n");
      break;
    }
  }
  return rc;
}

/* ======================================================================
FUNCTION
  get_cmp_handle

DESCRIPTION
  Get component handle.

PARAMETERS
  None

RETURN VALUE
  Error None.
========================================================================== */
void* get_cmp_handle(char *cmp_name)
{
  unsigned i    =0,j=0;

  DEBUG_PRINT("get_cmp_handle \n");
  for (i = 0; i < num_components; i++)
  {
    if (!strcmp(cmp_name, component[i].name))
    {
      for(j=0; j< OMX_COMP_MAX_INST; j++)
      {
        if (component[i].inst[j])
        {
          DEBUG_PRINT("get_cmp_handle match\n");
          return component[i].inst[j];
        }
      }
    }
  }
  DEBUG_PRINT("get_cmp_handle returning NULL \n");
  return NULL;
}

/* ======================================================================
FUNCTION
  OMX_DeInit

DESCRIPTION
  DeInitialize all the the relevant OMX components.

PARAMETERS
  None

RETURN VALUE
  Error None.
========================================================================== */
OMX_API OMX_ERRORTYPE OMX_APIENTRY
OMX_Deinit()
{
  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  OMX_GetHandle

DESCRIPTION
  Constructs requested component. Relevant library is loaded if needed.

PARAMETERS
  None

RETURN VALUE
  Error None  if everything goes fine.
========================================================================== */

 OMX_API OMX_ERRORTYPE OMX_APIENTRY
OMX_GetHandle(OMX_OUT OMX_HANDLETYPE*     handle,
              OMX_IN OMX_STRING    componentName,
              OMX_IN OMX_PTR             appData,
              OMX_IN OMX_CALLBACKTYPE* callBacks)
{
  OMX_ERRORTYPE  eRet = OMX_ErrorNone;
  int cmp_index = -1;
  int hnd_index = -1;
  int vpp_cmp_index = -1;

  DEBUG_PRINT("OMXCORE API :  GetHandle %p %s %p\n", handle,
                                                     componentName,
                                                     appData);
  pthread_mutex_lock(&lock_core);
  if(handle)
  {
    *handle = NULL;
    char optComponentName[OMX_MAX_STRINGNAME_SIZE];
    strlcpy(optComponentName, componentName, OMX_MAX_STRINGNAME_SIZE);

    if(strstr(componentName, "avc") && strstr(componentName, "decoder"))
    {
      void *libhandle = dlopen("libOmxVideoDSMode.so", RTLD_NOW);
      if(libhandle)
      {
        int (*fn_ptr)()  = dlsym(libhandle, "isDSModeActive");

        if(fn_ptr == NULL)
        {
          DEBUG_PRINT_ERROR("Error: isDSModeActive Not Found %s\n",
                    dlerror());
        }
        else
        {
          int isActive = fn_ptr();
          char *pSubString = strstr(componentName, ".dsmode");
          if(pSubString)
          {
            optComponentName[pSubString - componentName] = 0;
          }
          else if(isActive)
          {
            strlcat(optComponentName, ".dsmode", OMX_MAX_STRINGNAME_SIZE);
          }
          cmp_index = get_cmp_index(optComponentName);
        }
        dlclose(libhandle);
      }
      else
      {
        DEBUG_PRINT_ERROR("Failed to load dsmode library");
      }
    }

    if(cmp_index < 0)
    {
      cmp_index = get_cmp_index(componentName);
      strlcpy(optComponentName, componentName, OMX_MAX_STRINGNAME_SIZE);
    }
    if(cmp_index >= 0)
    {
      DEBUG_PRINT("getting fn pointer\n");

      // Load VPP omx component for decoder if vpp property is enabled
      const char *hwDecLib = "libOmxVdec.so";
      const char *swDecLib = "libOmxSwVdec.so";
      if (!strncmp(component[cmp_index].so_lib_name, hwDecLib, strlen(hwDecLib)) ||
          !strncmp(component[cmp_index].so_lib_name, swDecLib, strlen(swDecLib))) {
        bool isVppEnabled = false;
        bool isCSEnabled = false;
#ifndef VIDC_STUB_HAL
        isCSEnabled = isConfigStoreEnabled();
        if (isCSEnabled) {
          getConfigStoreBool("vpp", "enable", &isVppEnabled, false);
        }
#endif
        if (!isCSEnabled) {
          char value[PROPERTY_VALUE_MAX];
          if ((property_get("vendor.media.vpp.enable", value, NULL))
               && (!strcmp("1", value) || !strcmp("true", value))) {
            isVppEnabled = true;
          }
        }
        if (isVppEnabled) {
          DEBUG_PRINT("VPP property is enabled");
          vpp_cmp_index = get_cmp_index("OMX.qti.vdec.vpp");
          if (vpp_cmp_index < 0) {
            DEBUG_PRINT_ERROR("Unable to find VPP OMX lib in registry ");
          } else {
            DEBUG_PRINT("Loading vpp for vdec");
            cmp_index = vpp_cmp_index;
          }
        }
      }

       // dynamically load the so
      component[cmp_index].fn_ptr =
        omx_core_load_cmp_library(component[cmp_index].so_lib_name,
                                  &component[cmp_index].so_lib_handle);


      if(component[cmp_index].fn_ptr)
      {
        //Do not allow more than MAX limit for DSP audio decoders
        if((!strcmp(component[cmp_index].so_lib_name,"libOmxWmaDec.so")  ||
            !strcmp(component[cmp_index].so_lib_name,"libOmxAacDec.so")  ||
            !strcmp(component[cmp_index].so_lib_name,"libOmxG711Dec.so")  ||
            !strcmp(component[cmp_index].so_lib_name,"libOmxAlacDec.so") ||
            !strcmp(component[cmp_index].so_lib_name,"libOmxApeDec.so")) &&
            (number_of_adec_nt_session+1 > MAX_AUDIO_NT_SESSION)) {
            DEBUG_PRINT_ERROR("Rejecting new session..Reached max limit for DSP audio decoder session");
            pthread_mutex_unlock(&lock_core);
            return OMX_ErrorInsufficientResources;
        }
        // Construct the component requested
        // Function returns the opaque handle
        void* pThis = (*(component[cmp_index].fn_ptr))();
        if(pThis)
        {
          void *hComp = NULL;
          hComp = qc_omx_create_component_wrapper((OMX_PTR)pThis);
          if((eRet = qc_omx_component_init(hComp, optComponentName)) !=
                           OMX_ErrorNone)
          {
              DEBUG_PRINT("Component not created succesfully\n");
              pthread_mutex_unlock(&lock_core);
              return eRet;

          }
          qc_omx_component_set_callbacks(hComp,callBacks,appData);

          if (vpp_cmp_index >= 0)
          {
            hnd_index = get_comp_handle_index("OMX.qti.vdec.vpp");
          }
          else
          {
            hnd_index = get_comp_handle_index(optComponentName);
          }

          if(hnd_index >= 0)
          {
            component[cmp_index].inst[hnd_index]= *handle = (OMX_HANDLETYPE) hComp;
          }
          else
          {
            DEBUG_PRINT("OMX_GetHandle:NO free slot available to store Component Handle\n");
            pthread_mutex_unlock(&lock_core);
            return OMX_ErrorInsufficientResources;
          }
          DEBUG_PRINT("Component %p Successfully created\n",*handle);
          if(!strcmp(component[cmp_index].so_lib_name,"libOmxWmaDec.so")  ||
             !strcmp(component[cmp_index].so_lib_name,"libOmxAacDec.so")  ||
             !strcmp(component[cmp_index].so_lib_name,"libOmxG711Dec.so")  ||
             !strcmp(component[cmp_index].so_lib_name,"libOmxAlacDec.so") ||
             !strcmp(component[cmp_index].so_lib_name,"libOmxApeDec.so")) {

             number_of_adec_nt_session++;
             DEBUG_PRINT("OMX_GetHandle: number_of_adec_nt_session : %d\n",
                             number_of_adec_nt_session);
          }
        }
        else
        {
          eRet = OMX_ErrorInsufficientResources;
          DEBUG_PRINT("Component Creation failed\n");
        }
      }
      else
      {
        eRet = OMX_ErrorNotImplemented;
        DEBUG_PRINT("library couldnt return create instance fn\n");
      }

    }
    else
    {
      eRet = OMX_ErrorNotImplemented;
      DEBUG_PRINT("ERROR: Already another instance active  ;rejecting \n");
    }
  }
  else
  {
    eRet =  OMX_ErrorBadParameter;
    DEBUG_PRINT("\n OMX_GetHandle: NULL handle \n");
  }
  pthread_mutex_unlock(&lock_core);
  return eRet;
}
/* ======================================================================
FUNCTION
  OMX_FreeHandle

DESCRIPTION
  Destructs the component handles.

PARAMETERS
  None

RETURN VALUE
  Error None.
========================================================================== */
OMX_API OMX_ERRORTYPE OMX_APIENTRY
OMX_FreeHandle(OMX_IN OMX_HANDLETYPE hComp)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  int err = 0, i = 0;
  DEBUG_PRINT("OMXCORE API :  FreeHandle %p\n", hComp);

  // 0. Check that we have an active instance
  if((i=is_cmp_handle_exists(hComp)) >=0)
  {
    // 1. Delete the component
    if ((eRet = qc_omx_component_deinit(hComp)) == OMX_ErrorNone)
    {
        pthread_mutex_lock(&lock_core);
        clear_cmp_handle(hComp);
        /* Unload component library */
    if( (i < (int)num_components) && component[i].so_lib_handle)
    {
           if(check_lib_unload(i))
           {
              DEBUG_PRINT_ERROR(" Unloading the dynamic library for %s\n",
                                  component[i].name);
              err = dlclose(component[i].so_lib_handle);
              if(err)
              {
                  DEBUG_PRINT_ERROR("Error %d in dlclose of lib %s\n",
                                     err,component[i].name);
              }
              component[i].so_lib_handle = NULL;
           }
           if(!strcmp(component[i].so_lib_name,"libOmxWmaDec.so")  ||
              !strcmp(component[i].so_lib_name,"libOmxAacDec.so")  ||
              !strcmp(component[i].so_lib_name,"libOmxAlacDec.so") ||
              !strcmp(component[i].so_lib_name,"libOmxApeDec.so")) {
               if(number_of_adec_nt_session>0)
                   number_of_adec_nt_session--;
               DEBUG_PRINT_ERROR("OMX_FreeHandle: reduced number_of_adec_nt_session %d\n",
                                   number_of_adec_nt_session);
           }
    }
    pthread_mutex_unlock(&lock_core);
    }
    else
    {
        DEBUG_PRINT(" OMX_FreeHandle failed on %p\n", hComp);
        return eRet;
    }
  }
  else
  {
    DEBUG_PRINT_ERROR("OMXCORE Warning: Free Handle called with no active instances\n");
  }
  return OMX_ErrorNone;
}
/* ======================================================================
FUNCTION
  OMX_SetupTunnel

DESCRIPTION
  Not Implemented.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
OMX_API OMX_ERRORTYPE OMX_APIENTRY
OMX_SetupTunnel(OMX_IN OMX_HANDLETYPE outputComponent,
                OMX_IN OMX_U32             outputPort,
                OMX_IN OMX_HANDLETYPE  inputComponent,
                OMX_IN OMX_U32              inputPort)
{
  (void) outputComponent, (void) outputPort, (void) inputComponent, (void) inputPort;
  /* Not supported right now */
  DEBUG_PRINT("OMXCORE API: OMX_SetupTunnel Not implemented \n");
  return OMX_ErrorNotImplemented;
}
/* ======================================================================
FUNCTION
  OMX_GetContentPipe

DESCRIPTION
  Not Implemented.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
OMX_API OMX_ERRORTYPE
OMX_GetContentPipe(OMX_OUT OMX_HANDLETYPE* pipe,
                   OMX_IN OMX_STRING        uri)
{
  (void) pipe, (void) uri;
  /* Not supported right now */
  DEBUG_PRINT("OMXCORE API: OMX_GetContentPipe Not implemented \n");
  return OMX_ErrorNotImplemented;
}

/* ======================================================================
FUNCTION
  OMX_GetComponentNameEnum

DESCRIPTION
  Returns the component name associated with the index.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
OMX_API OMX_ERRORTYPE OMX_APIENTRY
OMX_ComponentNameEnum(OMX_OUT OMX_STRING componentName,
                      OMX_IN  OMX_U32          nameLen,
                      OMX_IN  OMX_U32            index)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  DEBUG_PRINT("OMXCORE API - OMX_ComponentNameEnum %p %d %d\n", componentName
                                                              ,(unsigned)nameLen
                                                              ,(unsigned)index);
  if (index < num_components &&
      strncmp(component[index].name, "OMX.QCOM.CUST.COMP.START",
              strlen("OMX.QCOM.CUST.COMP.START")))
  {
    #ifdef _ANDROID_
    strlcpy(componentName, component[index].name, nameLen);
    #else
    strlcpy(componentName, component[index].name,nameLen);
    #endif
  }
  else
  {
    eRet = OMX_ErrorNoMore;
  }
  return eRet;
}

/* ======================================================================
FUNCTION
  OMX_GetComponentsOfRole

DESCRIPTION
  Returns the component name which can fulfill the roles passed in the
  argument.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
OMX_API OMX_ERRORTYPE
OMX_GetComponentsOfRole(OMX_IN OMX_STRING      role,
                        OMX_INOUT OMX_U32* numComps,
                        OMX_INOUT OMX_U8** compNames)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  unsigned i,j,namecount=0;

  printf(" Inside OMX_GetComponentsOfRole \n");

  /*If CompNames is NULL then return*/
  if (compNames == NULL)
  {
    if (numComps == NULL)
    {
      eRet = OMX_ErrorBadParameter;
    }
    else
    {
      *numComps = 0;
      for (i = 0; i < num_components; i++)
      {
        for (j = 0; j < OMX_CORE_MAX_CMP_ROLES && component[i].roles[j]; j++)
        {
          if (!strcmp(role,component[i].roles[j]))
          {
            (*numComps)++;
          }
        }
      }
    }
    return eRet;
  }

  if(numComps)
  {
      namecount = *numComps;

      if (namecount == 0)
      {
          return OMX_ErrorBadParameter;
      }

    *numComps          = 0;

    for (i = 0; i < num_components;i++)
    {
      for (j = 0; j < OMX_CORE_MAX_CMP_ROLES && component[i].roles[j]; j++)
      {
        if (!strcmp(role,component[i].roles[j]))
        {
          #ifdef _ANDROID_
          strlcpy((char *)compNames[*numComps],component[i].name, OMX_MAX_STRINGNAME_SIZE);
          #else
          strlcpy((char *)compNames[*numComps],component[i].name, OMX_MAX_STRINGNAME_SIZE);
          #endif
          (*numComps)++;
          break;
        }
      }
          if (*numComps == namecount)
          {
          break;
        }
    }
  }
  else
  {
    eRet = OMX_ErrorBadParameter;
  }

  printf(" Leaving OMX_GetComponentsOfRole \n");
  return eRet;
}
/* ======================================================================
FUNCTION
  OMX_GetRolesOfComponent

DESCRIPTION
  Returns the primary role of the components supported.

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
OMX_API OMX_ERRORTYPE
OMX_GetRolesOfComponent(OMX_IN OMX_STRING compName,
                        OMX_INOUT OMX_U32* numRoles,
                        OMX_OUT OMX_U8** roles)
{
  /* Not supported right now */
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  unsigned i,j,numofroles = 0;;
  DEBUG_PRINT("GetRolesOfComponent %s\n",compName);

  if (roles == NULL)
  {
    if (numRoles == NULL)
    {
      eRet = OMX_ErrorBadParameter;
    }
    else
    {
      *numRoles = 0;
      for (i = 0; i < num_components; i++)
      {
        if (!strcmp(compName,component[i].name))
        {
          for (j = 0; j < OMX_CORE_MAX_CMP_ROLES && component[i].roles[j]; j++)
          {
            (*numRoles)++;
          }
          break;
        }
      }
    }
    return eRet;
  }

  if(numRoles)
  {
    if (*numRoles == 0)
    {
        return OMX_ErrorBadParameter;
    }

    numofroles = *numRoles;
    *numRoles = 0;
    for (i = 0; i < num_components; i++)
    {
      if (!strcmp(compName, component[i].name))
      {
        for(j=0; (j<OMX_CORE_MAX_CMP_ROLES) && component[i].roles[j];j++)
        {
          if(roles && roles[*numRoles])
          {
            #ifdef _ANDROID_
            strlcpy((char *)roles[*numRoles],component[i].roles[j],OMX_MAX_STRINGNAME_SIZE);
            #else
            strlcpy((char *)roles[*numRoles],component[i].roles[j],OMX_MAX_STRINGNAME_SIZE);
            #endif
          }
          (*numRoles)++;
          if (numofroles == *numRoles)
          {
              break;
          }
        }
        break;
      }
    }
  }
  else
  {
    DEBUG_PRINT("ERROR: Both Roles and numRoles Invalid\n");
    eRet = OMX_ErrorBadParameter;
  }
  return eRet;
}

OMX_API OMX_BOOL
OMXConfigParser(
    OMX_PTR aInputParameters,
    OMX_PTR aOutputParameters)
{
    OMX_BOOL Status = OMX_TRUE;
    VideoOMXConfigParserOutputs *aOmxOutputParameters;
    OMXConfigParserInputs *aOmxInputParameters;
    aOmxOutputParameters = (VideoOMXConfigParserOutputs *)aOutputParameters;
    aOmxInputParameters = (OMXConfigParserInputs *)aInputParameters;

    aOmxOutputParameters->width = 176; //setting width to QCIF
    aOmxOutputParameters->height = 144; //setting height to QCIF

    //TODO
    //Qcom component do not use the level/profile from IL client .They are parsing the first buffer
    //sent in ETB so for now setting the defalut values . Going farward we can call
    //QC parser here.
    if (0 == strcmp(aOmxInputParameters->cComponentRole, (OMX_STRING)"video_decoder.avc"))
    {
       aOmxOutputParameters->profile = 66; //minimum supported h264 profile - setting to baseline profile
       aOmxOutputParameters->level = 0;  // minimum supported h264 level
    }
    else if ((0 == strcmp(aOmxInputParameters->cComponentRole, (OMX_STRING)"video_decoder.mpeg4")) || (0 == strcmp(aOmxInputParameters ->cComponentRole, (OMX_STRING)"video_decoder.h263")))
    {
       aOmxOutputParameters->profile = 8; //minimum supported h263/mpeg4 profile
       aOmxOutputParameters->level = 0; // minimum supported h263/mpeg4 level
    }

    return Status;
}
