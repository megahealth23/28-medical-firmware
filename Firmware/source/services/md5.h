/* Copyright (c) 2016 MegaHealth. All Rights Reserved.
 * md5.h
 * AUTHOR:zhao mengshou
 * DATE:2016-11-11 15:20
 * http://www.megahealth.cn
 *
 */

#ifndef MD5_H  
#define MD5_H  
  
#ifdef __cplusplus  
extern "C"  
{  
#endif  

void md5(const uint8_t *initial_msg, size_t initial_len, uint8_t *digest);

#ifdef __cplusplus  
}  
#endif  
#endif /* MD5_H */  



