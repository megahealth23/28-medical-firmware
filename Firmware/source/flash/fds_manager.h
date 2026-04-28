#ifndef __FDS_MANAGER_H__
#define __FDS_MANAGER_H__
#include <sdk_common.h>
#include "fds.h"


typedef enum e_fds_status{
    e_fds_no_init,
    e_fds_initializing,
    e_fds_idle,
    e_fds_writing,
    e_fds_updating,
    e_fds_deleting,
    e_fds_gcing
} E_FDS_STATUS;

typedef void (*fds_m_handler)(fds_evt_t const *p_evt);

ret_code_t fds_m_init(void);
ret_code_t fds_m_write(uint8_t* data, uint32_t length, uint16_t file_id, uint16_t key);
ret_code_t fds_m_del_file(uint16_t file_id);
ret_code_t fds_m_read(uint16_t file_id, uint16_t key, uint8_t* pData, uint32_t* pLength);
ret_code_t fds_m_find(uint16_t file_id, uint16_t key);
ret_code_t fds_m_gc();
uint32_t fds_m_free_space();
uint16_t fds_file_last_key(uint16_t file_id);

#endif