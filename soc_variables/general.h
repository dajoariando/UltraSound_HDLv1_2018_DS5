// offsets for output control signal
#define MUX_LE_OFST (10)
#define MUX_SET_OFST (9)
#define MUX_CLR_OFST (8)
#define FSM_RST_OFST (7)
#define sw_off_OFST (6)
#define tx_path_en_OFST (5)
#define pulser_en_OFST (4)
#define lm96570_pin_reset_OFST (3)
#define lm96570_tx_en_OFST (2)
#define lm96570_spi_reset_OFST (1)
#define lm96570_start_OFST (0)

// mask for output control signal
#define MUX_LE_MSK (1<<MUX_LE_OFST)
#define MUX_SET_MSK (1<<MUX_SET_OFST)
#define MUX_CLR_MSK (1<<MUX_CLR_OFST)
#define FSM_RST_MSK (1<<FSM_RST_OFST )
#define sw_off_MSK (1<<sw_off_OFST )
#define tx_path_en_MSK (1<<tx_path_en_OFST )
#define pulser_en_MSK (1<<pulser_en_OFST )
#define lm96570_pin_reset_MSK (1<<lm96570_pin_reset_OFST )
#define lm96570_tx_en_MSK (1<<lm96570_tx_en_OFST )
#define lm96570_spi_reset_MSK (1<<lm96570_spi_reset_OFST )
#define lm96570_start_MSK (1<<lm96570_start_OFST )

// default for output control signal
#define CNT_OUT_DEFAULT 0b000000
