#ifndef FTM_COMMON_H_
#define FTM_COMMON_H_

/*****************************************************************************
 * version
 *****************************************************************************/
#define FTMAPIVersion "0480"
/*****************************************************************************
 * FIXME
 *****************************************************************************/
//return
#define ERROR     -1
#define OK         0
#define TIMEOUT    1
#define NOTSUPPORT 2
#define PARAMETER_ERROR 3
//cmd
#define CMD_PARAM_SIZE  128 //32
#define CMD_RESULT_SIZE 128//64

/*****************************************************************************
 * FIXME: ftm dispatcher
 *****************************************************************************/
#define FTMD_CMD_LINE_MAX_SIZE 192

/* ftm command queue sate */
#define FTMD_CMD_STATE_EMPTY 	0
#define FTMD_CMD_STATE_QUEUED   1
#define FTMD_CMD_STATE_WIP      2

/* indicate onyx product SW library versions */

/*****************************************************************************
 * FIXME: ftm memory dump
 *****************************************************************************/
#define FTMD_RX_DUMP_SIZE 		(1024)
#define FTMD_CR 0x0d
#define FTMD_LF 0x0a

/*****************************************************************************
 * FIXME: ftm flags
 *****************************************************************************/
#define FTM_BLT 0
#define FTM_SLT 1

/*****************************************************************************
 * FIXME: ftm debug flags
 *****************************************************************************/
#define FTMD_PARSER_DBG 	0

/*****************************************************************************
 * FIXME: ftm dispatcher
 *****************************************************************************/
/* ftm state variables, ftm_cmd_state and ftm_cmd_line buffer */

struct ftm_state_s
{
    int	ftm_cmd_state;
    char ftm_cmd_line[FTMD_CMD_LINE_MAX_SIZE];
};

#endif /* FTM_COMMON_H_ */
