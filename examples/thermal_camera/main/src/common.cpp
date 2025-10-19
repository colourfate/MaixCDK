#include "common.h"
#include <stdlib.h>

void common_init(void)
{
    char log_lvl;
    char *log_env;

    /* close printf buffer */
    setbuf(stdout, NULL);
    /* initialize EasyLogger */
    elog_init();

    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_LVL | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_LVL | ELOG_FMT_FUNC | ELOG_FMT_LINE);

	log_env = getenv("ELOG_LVL");
    if (log_env == NULL) {
        log_lvl = 'I';
    } else {
        log_lvl = log_env[0];
    }
    switch (log_lvl)
    {
    case 'A':
        elog_set_filter_lvl(ELOG_LVL_ASSERT);
        break;
    case 'E':
        elog_set_filter_lvl(ELOG_LVL_ERROR);
        break;
    case 'W':
        elog_set_filter_lvl(ELOG_LVL_WARN);
        break;
    case 'I':
        elog_set_filter_lvl(ELOG_LVL_INFO);
        break;
    case 'D':
        elog_set_filter_lvl(ELOG_LVL_DEBUG);
        break;
    case 'V':
        elog_set_filter_lvl(ELOG_LVL_VERBOSE);
        break;
    default:
        elog_set_filter_lvl(ELOG_LVL_INFO);
        break;
    }
#ifdef ELOG_COLOR_ENABLE
    elog_set_text_color_enabled(true);
#endif
    /* start EasyLogger */
    elog_start();
}