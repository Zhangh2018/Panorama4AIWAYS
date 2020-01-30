/******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2017-2018 NXP;
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dirent.h>
#include <devctl.h>
#include <semaphore.h>
#include <fcntl.h>

#include <hw/inout.h>
#include <sys/dispatch.h>
#include <sys/iofunc.h>
#include <sys/procmgr.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <errno.h>


#include "qnx/devctl_cmd.h"
#include "oal_debug_out.h"
#include "oal_allocation_kernel.h"
#include "oal_cma_list.h"
#include "vdb_log.h"            /* logging; VDB_LOG_ERROR */
#include "common_helpers.h"     /* UNUSED_ARG macro */


/* Module devices bookkeeping */
uint8_t  oal_devices_loaded = 0;
uint8_t  oal_devices_autobalanced = 0;
uint32_t oal_device_alignment[OAL_MAX_ALLOCATOR_NUM] = {0};

/* Semaphore structs & macros */
static sem_t oal_short_term_sema;
static sem_t oal_long_term_sema;

#define MSG_MAX_SIZE    		2048
#define NPARTS_MAX      		1

#define MIN_AVAILABLE_THREADS		2
#define MAX_AVAILABLE_THREADS		4
#define THREAD_INCREMENT 		1
#define MAX_THREADS			50

#define OBTAIN_SHORT_LOCK() {                                               \
                              if (sem_wait(&oal_short_term_sema))           \
                              {                                             \
                              }                                             \
                            }

#define RELEASE_SHORT_LOCK() {                                              \
                               sem_post(&oal_short_term_sema);              \
                             }

#define OBTAIN_LONG_LOCK() {                                                \
                              if (sem_wait(&oal_long_term_sema))            \
                              {                                             \
                              }                                             \
                           }

#define RELEASE_LONG_LOCK() {                                               \
                               sem_post(&oal_long_term_sema);               \
                            }

#define OAL_DEVICE "/dev/oal"

//TODO: read info about memory zones using FDT parse
static oal_mem memory_zones[OAL_MAX_ALLOCATOR_NUM] = {
        {"/memory/below4G/ram/oalddr0", 0x8B000000, 0x05000000, 0, 0x1000, 0 ,1},
        {"/memory/below4G/ram/oalddr1", 0xCB000000, 0x05000000, 1, 0x1000, 0, 1},
        {"/memory/below4G/oalsramS", 0x3E800000, 0x00300000, 2, 0x8, 1, 0},
        {"/memory/below4G/oalsramM", 0x3EB00000, 0x00100000, 3, 0x8, 1, 0}
};

static int oal_mem_device_io_devctl (resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
    int attr_unlock_ret;
    int sts;
    void *data;
    int nbytes;
    int dcmd;
    pid_t pid;
    int retval = 0;

    do {
        /* check standard POSIX-supported devctl() */
        if ((sts = iofunc_devctl_default (ctp, msg, ocb)) != _RESMGR_DEFAULT)
        {
            retval = sts;
            continue;
        }
        attr_unlock_ret = iofunc_unlock_ocb_default(ctp,msg,ocb);
        if (attr_unlock_ret) {
            retval = attr_unlock_ret;
            continue;
        }

        pid = ctp->info.pid;

        /* assign a pointer to the data area of the message */
        data = _DEVCTL_DATA (msg->i);

        /* get other data that can't be wiped out before we wipe the data structure */
        dcmd = msg->i.dcmd;

        /* preset the number of bytes that we'll return to zero */
        nbytes = 0;

        switch (dcmd)
        {

            case CMD_MUTEX:
            {
                OBTAIN_LONG_LOCK();
                break;
            }

            case CMD_UNMUTEX:
            {
                RELEASE_LONG_LOCK();
                break;
            }

            case CMD_ALLOC:
            {
                CMD_ALLOC_TYPE *aux = (CMD_ALLOC_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                aux->ret_phys_pointer = oal_alloc(aux->size, aux->align, aux->chunk_id, pid);
                nbytes = sizeof(CMD_ALLOC_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_FREE:
            {
                uint64_t *handle_pointer = (uint64_t *)data;

                OBTAIN_SHORT_LOCK();

                *handle_pointer = oal_free_phys(*handle_pointer);
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_SIZE:
            {
                uint64_t *handle_pointer = (uint64_t *)data;

                OBTAIN_SHORT_LOCK();

                *handle_pointer = oal_getsize(*handle_pointer);
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_BUFFER:
            {
                CMD_BUFFER_TYPE *aux = (CMD_BUFFER_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                aux->ret_virt_pointer = oal_getbuffer(aux->handle_pointer, aux->type);
                nbytes = sizeof(CMD_BUFFER_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MAP:
            {
                CMD_MAP_TYPE *aux = (CMD_MAP_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                aux->virtual_pointer = oal_map(aux->handle_pointer, aux->type, aux->virtual_pointer);
                nbytes = sizeof(CMD_MAP_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MAP_PHYS:
            {
                CMD_MAP_TYPE *aux = (CMD_MAP_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                aux->virtual_pointer = oal_map_phys(aux->handle_pointer, aux->type, aux->virtual_pointer);
                nbytes = sizeof(CMD_MAP_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MAP_NEW:
            {
                CMD_MAP_NEW_TYPE *aux = (CMD_MAP_NEW_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                aux->virtual_pointer = oal_map_new(aux->type, aux->virtual_pointer, aux->size, pid);
                nbytes = sizeof(CMD_MAP_NEW_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_INFO:
            {
                uint64_t *allocs = (uint64_t *)data;

                OBTAIN_SHORT_LOCK();

                *allocs = oal_get_num_allocations();
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_INFO_MAP:
            {
                uint64_t *allocs = (uint64_t *)data;

                OBTAIN_SHORT_LOCK();

                *allocs = oal_get_num_mappings();
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MAPPINGS:
            {
                CMD_MAPPINGS_TYPE *aux = (CMD_MAPPINGS_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                oal_getmappings(aux->address[0], aux->address);
                nbytes = sizeof(CMD_MAPPINGS_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MAPPINGSHW:
            {
                CMD_MAPPINGS_TYPE *aux = (CMD_MAPPINGS_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                oal_getmappingshw(aux->address[0], aux->address);
                nbytes = sizeof(CMD_MAPPINGS_TYPE);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_UNMAP:
            {
                uint64_t *handle_pointer  = (uint64_t *)data;

                OBTAIN_SHORT_LOCK();

                *handle_pointer = oal_unmap(*handle_pointer);
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MEMORY_SIZE_TOTAL_GET:
            {
                int64_t *lSizeInBytes = (int64_t *)data;

                OBTAIN_SHORT_LOCK();

                *lSizeInBytes = oal_memorysizetotal();
                nbytes = sizeof(int64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MEMORY_SIZE_FREE_GET:
            {
                int64_t *lSizeInBytes = (int64_t *)data;

                OBTAIN_SHORT_LOCK();

                *lSizeInBytes = oal_memorysizefree();
                nbytes = sizeof(int64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MEMORY_GET_DEVICES:
            {
                uint8_t *lSizeInBytes = (uint8_t *)data;

                OBTAIN_SHORT_LOCK();

                *lSizeInBytes = oal_devices_loaded;
                nbytes = sizeof(uint8_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MEMORY_GET_AUTOBALANCE:
            {
                uint8_t *lSizeInBytes = (uint8_t *)data;

                OBTAIN_SHORT_LOCK();

                *lSizeInBytes = oal_devices_autobalanced;
                nbytes = sizeof(uint8_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MEMORY_GET_BASE:
            {
                uint64_t *aux = (uint64_t *)data;

                OBTAIN_SHORT_LOCK();

                *aux = apex_allocator_get_physical_base(*aux);
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_MEMORY_GET_SIZE:
            {
                int64_t mem_size;
                uint8_t chunk_id = *(uint8_t *)data;
                int64_t *aux = (int64_t *)data;

                OBTAIN_SHORT_LOCK();

                mem_size = apex_allocator_get_total_size(chunk_id);
                memcpy(aux, &mem_size, sizeof(int64_t));
                nbytes = sizeof(uint64_t);

                RELEASE_SHORT_LOCK();

                break;
            }

            case CMD_AREA_INFO_GET:
            {
                CMD_ALLOC_TYPE *aux = (CMD_ALLOC_TYPE *)data;

                OBTAIN_SHORT_LOCK();

                aux->chunk_id = oal_get_chunkid(aux->ret_phys_pointer);
                nbytes = sizeof(CMD_ALLOC_TYPE);

                RELEASE_SHORT_LOCK();

                /* check for valid chunk ID */
                if (aux->chunk_id > OAL_MAX_ALLOCATOR_NUM) {
                    retval = EAGAIN;
                }
                break;
            }

            case CMD_MEMORY_ZONES_GET:
            {
                oal_mem *aux = (oal_mem *)data;

                OBTAIN_SHORT_LOCK();

                memcpy(aux, &memory_zones, sizeof(memory_zones));
                nbytes = sizeof(memory_zones);

                RELEASE_SHORT_LOCK();

                break;
            }

            default:
            {
                VDB_LOG_ERROR("OAL: UNKNOWN OPERATION\n");
                retval = ENOTTY;
                break;
            }
        }

        if (!retval)
        {
            /* return data (if any) to the client */
            memset (&msg->o, 0, sizeof(msg -> o));
            msg->o.nbytes = nbytes;
            SETIOV (ctp->iov, &msg->o, sizeof(msg->o) + nbytes);

            retval = _RESMGR_NPARTS (1);
        }
    } while (0);

    return retval;
}

typedef THREAD_POOL_PARAM_T *(*attr_block_func_t)(THREAD_POOL_PARAM_T *ctp);
typedef void (*attr_unblock_func_t)(THREAD_POOL_PARAM_T *ctp);
typedef int (*attr_handler_func_t)(THREAD_POOL_PARAM_T *ctp);
typedef THREAD_POOL_PARAM_T *(*attr_context_alloc_t)(THREAD_POOL_HANDLE_T *dpp);
typedef void (*attr_context_free_t)(THREAD_POOL_PARAM_T *ctp);

int main(int argc, char **argv)
{
    resmgr_attr_t           resmgr_attr;
    resmgr_connect_funcs_t  connect_func;
    resmgr_io_funcs_t       io_func;
    iofunc_attr_t           attr;
    dispatch_t              *dpp;
    thread_pool_attr_t      threadpool_attribs;
    thread_pool_t           *thread_pool;

    int      retval = EXIT_FAILURE;
    uint8_t  oal_memory_zones = 0;
    uint8_t  oal_auto, oal_init, oal_id;
    uint32_t oal_align;
    uint64_t i;
    uint64_t oal_mem_start, oal_mem_len;

    UNUSED_ARG(argc);
    UNUSED_ARG(argv);

    vdb_LogInit();      /* enable logging */

    do {
        /* initialize semaphores */
        if (sem_init(&oal_short_term_sema, 0, 1))
        {
            VDB_LOG_ERROR("Failed to initialize short term semaphore\n");
            continue;
        }

        if (sem_init(&oal_long_term_sema, 0, 1))
        {
            VDB_LOG_ERROR("Failed to initialize long term semaphore\n");
            continue;
        }

        for (i = 0; i < OAL_MAX_ALLOCATOR_NUM; i++)
        {
            /* skip if entry is 0 */
            if (is_memory_zone_empty(memory_zones[i])) {
                continue;
            }

            /* increment oal memory zones */
            oal_memory_zones++;

            /* get memory zone info */
            oal_mem_start = memory_zones[i].start_address;
            oal_mem_len = memory_zones[i].size;
            oal_id = memory_zones[i].id;
            oal_align = memory_zones[i].align;
            oal_init = memory_zones[i].init;
            oal_auto = memory_zones[i].autobalance;


            oal_devices_loaded |= (1U << oal_id);
            oal_device_alignment[oal_id] = oal_align;

            /* get if the memory is autobalanced */
            if (!(bool)oal_auto) {
                oal_devices_autobalanced &= ~(1U << oal_id);
            }
            else {
                oal_devices_autobalanced |= 1U << oal_id;
            }

            /* init allocator */
            apex_allocator_mmap_init(oal_id, oal_mem_start, oal_mem_len);

            // TODO: initialize memory zone with zeros - doesn't work for now
            #if 0
            if (oal_init)
            {
                mem_zone_fd = posix_typed_mem_open(memory_zones[i].name, O_RDWR, 0);
                if (mem_zone_fd < 0)
                {
                    perror ("Failed to open address space\n");
                    exit (EXIT_FAILURE);
                }

                mem_zone_addr = mmap(0, oal_mem_len, PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED, mem_zone_fd, oal_mem_start);
                if (mem_zone_addr == MAP_FAILED)
                {
                    perror ("Failed to map memory\n");
                    exit (EXIT_FAILURE);
                }

                //TODO: clear memory - board restarts here
                memset(mem_zone_addr, 0, oal_mem_len);

                //clear resources
                munmap(mem_zone_addr, oal_mem_len);
                close(mem_zone_fd);
            }
            #endif

            VDB_LOG_NOTE("OAL region successfully mapped %lX@%lX, Alignment: 0x%X\n", oal_mem_len, oal_mem_start, oal_device_alignment[oal_id]);
            if (oal_init)
            {
                VDB_LOG_NOTE("Initialized to 0x0ULL\n");
            }

            /* init cma list */
            cma_list_init(oal_id, oal_mem_start, oal_mem_start + oal_mem_len, oal_device_alignment[oal_id]);
            VDB_LOG_NOTE("CMA: ID %02X. Start %02lX. End %02lX. Length %02lX. Num %02lX.\n",
                    oal_id, oal_mem_start, (oal_mem_start + oal_mem_len), oal_mem_len, (oal_mem_len / oal_device_alignment[oal_id]));
        }

        dpp = dispatch_create ();
        if (dpp == NULL)
        {
            VDB_LOG_ERROR ("Unable to dispatch_create()\n");
            continue;
        }

        /* bind default functions into the outcall tables */
        iofunc_func_init (_RESMGR_CONNECT_NFUNCS, &connect_func, _RESMGR_IO_NFUNCS, &io_func);
        iofunc_attr_init (&attr, S_IFREG | 0666, 0, 0);

        /* override "io_func" function */
        io_func.devctl = oal_mem_device_io_devctl;

        /* initialize the various data structures */
        memset (&resmgr_attr, 0, sizeof (resmgr_attr));
        /* configure message attributes */
        resmgr_attr.nparts_max = NPARTS_MAX;
        /* 2KB should be enough our data structures */
        resmgr_attr.msg_max_size = MSG_MAX_SIZE;

        /* establish a name in the pathname space */
        if (resmgr_attach (dpp, &resmgr_attr, OAL_DEVICE, _FTYPE_ANY, 0, &connect_func, &io_func, &attr) == -1)
        {
            VDB_LOG_ERROR ("Unable to resmgr_attach\n");
            continue;
        }

        /* set permissions for io_devctl */
        if (ThreadCtl (_NTO_TCTL_IO, NULL) == -1)
        {
            VDB_LOG_ERROR ("Failed to get control\n");
            continue;
        }

        memset(&threadpool_attribs, 0, sizeof(threadpool_attribs));
        threadpool_attribs.handle = dpp;
        threadpool_attribs.context_alloc = (attr_context_alloc_t)dispatch_context_alloc;
        threadpool_attribs.block_func = (attr_block_func_t)dispatch_block;
        threadpool_attribs.unblock_func = (attr_unblock_func_t)dispatch_unblock;
        threadpool_attribs.handler_func = (attr_handler_func_t)dispatch_handler;
        threadpool_attribs.context_free = (attr_context_free_t)dispatch_context_free;
        threadpool_attribs.lo_water = MIN_AVAILABLE_THREADS;
        threadpool_attribs.hi_water = MAX_AVAILABLE_THREADS;
        threadpool_attribs.increment = THREAD_INCREMENT;
        threadpool_attribs.maximum = MAX_THREADS;

        /* Creating thread pool */
        thread_pool = thread_pool_create(&threadpool_attribs, POOL_FLAG_EXIT_SELF);
        if(thread_pool == NULL) {
            VDB_LOG_ERROR ("%s: Unable to initialize thread pool.\n", argv[0]);
            continue;
        }

        /* Start thread pool */
        thread_pool_start(thread_pool);
        retval = EXIT_SUCCESS;
    } while (0);

    return retval;
}
