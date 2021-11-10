/**
 * @file  macros.h
 * @date  10-December-2020
 * @brief Useful macros definitions.
 *
 * @author
 * @author
 */

#ifndef MACROS_H
#define MACROS_H

/***** FIRMWARE VERSION ***************************************************** */
#define COMMON_MACROS_VER_MAJOR                                             2020
#define COMMON_MACROS_VER_MINOR                                               12
#define COMMON_MACROS_VER_PATCH                                                1


/*
 * @brief Turns to one in REG the bits set to one in MASK.
 */
#ifndef SET_MASK
#define SET_MASK(REG, MASK)     ((REG) |= (MASK))
#endif

/*
 * @brief Turns to zero in REG the bits set to one in MASK.
 */
#ifndef CLEAR_MASK
#define CLEAR_MASK(REG, MASK)   ((REG) &= ~(MASK))
#endif

/*
 * @brief Toggles in REG the bits set to one in MASK.
 */
#ifndef TOGGLE_MASK
#define TOGGLE_MASK(REG, MASK)   ((REG) ^= (MASK))
#endif

/*
 * @brief Returns the state of the bits in REG for the bits set to one in MASK.
 */
#ifndef READ_MASK
#define READ_MASK(REG, MASK)    ((REG) & (MASK))
#endif

/*
 * @brief Assigns 0 to all bits on REG.
 */
#ifndef CLEAR_REG
#define CLEAR_REG(REG)        ((REG) = (0x0))
#endif

/*
 * @brief Assigns VAL to REG.
 */
#ifndef WRITE_REG
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#endif

/*
 * @brief Returns the value stored in REG.
 */
#ifndef READ_REG
#define READ_REG(REG)         ((REG))
#endif

/*
 * @brief Applies a clear and a set mask to REG.
 */
#ifndef MODIFY_REG
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#endif

/*
 * @brief This comment is a placeholder.
 */
#ifndef POSITION_VAL
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))
#endif

/*
 * @brief Turns to one the REG bit in position NUM.
 */
#ifndef SET_BIT_NUM
#define SET_BIT_NUM(REG, NUM)     SET_MASK(REG, (1UL << (NUM)))
#endif

/*
 * @brief Turns to zero the REG bit in position NUM.
 */
#ifndef CLEAR_BIT_NUM
#define CLEAR_BIT_NUM(REG, NUM)   CLEAR_MASK(REG, (1UL << (NUM)))
#endif

/*
 * @brief Toggles the value of REG bit in position NUM.
 */
#ifndef TOGGLE_BIT_NUM
#define TOGGLE_BIT_NUM(REG, NUM)  TOGGLE_MASK(REG, (1UL << (NUM)))
#endif

/*
 * @brief Reads without shifting the value of REG bit in position NUM.
 */
#ifndef READ_BIT_NUM
#define READ_BIT_NUM(REG,NUM)     READ_MASK(REG, (1UL << (NUM)))
#endif
/*
 * @brief Initializes an array with the specified value
 */
#ifndef INIT_ARRAY
#define INIT_ARRAY(SIZE,VAL)  { [0 ... (SIZE - 1) ] = VAL }
#endif

#endif /* MACROS_H */
