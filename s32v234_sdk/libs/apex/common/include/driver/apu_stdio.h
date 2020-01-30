/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2018 NXP Semiconductor;
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
****************************************************************************/

#ifndef APU_STDIO_H
#define APU_STDIO_H

#include <ctype.h>
#include <string.h>
#include <stdarg.h>

/* Define this before including apu_stdio.h */
/* Maximum size: 64K, but also restricted by DMEM size */
#ifndef APEX_LOG_SET_SIZE
#define APEX_LOG_SET_SIZE 4096
#endif

uint16_t APEX_LOG_SIZE = APEX_LOG_SET_SIZE;
char APEX_LOG_BUFFER[APEX_LOG_SET_SIZE];
uint16_t APEX_LOG_INDEX = 0;

#define PAD_ZEROES     (1 << 0)
#define SIGNED         (1 << 1)
#define PLUS           (1 << 2)
#define SPACE          (1 << 3)
#define LEFT_JUSTIFIED (1 << 4)
#define SPECIAL        (1 << 5)
#define LOWER_CASE     (1 << 6)

inline void cwrite(char** str, char c, int* max_characters)
{
    if(*max_characters > 0) {
        *((*str)++) = c;
        (*max_characters)--;
    }
}

/* For base larger than 16, change these lines */
#define MAX_BASE 16
const char *digits_large = "0123456789ABCDEF";
const char *digits_small = "0123456789abcdef";

static char* num_to_base(char* str, int* max_characters, int num, int base, int size, int precision, int type)
{
    char c;
    char sign;
    #define MAX_DIGITS 32
    char tmp[MAX_DIGITS]; /* Largest size is 32 digits for base 2 */
    int i;
    const char *digits;
    char *buf = str;

    if (base < 2 || base > MAX_BASE)
        return str;

    digits = (type & LOWER_CASE) ? digits_small : digits_large;

    if (type & LEFT_JUSTIFIED)
        type &= ~PAD_ZEROES;

    c = (type & PAD_ZEROES) ? '0' : ' ';

    if (type & SIGNED && num < 0)
    {
        sign = '-';
        num = -num;
    }
    else
        sign = (type & PLUS) ? '+' : ((type & SPACE) ? ' ' : 0);

    if (sign)
        size--;
    if (type & SPECIAL)
        size = (base == 16) ? (size - 2) : ((base == 8) ? (size - 1) : size);

    i = 0;
    if (num == 0)
        tmp[i++] = '0';
    else
        while (num != 0 && i < MAX_DIGITS)
        {
            tmp[i++] = digits[num % base];
            num /= base;
        }

    precision = (i > precision) ? i : precision;
    size -= precision;

    if (!(type & (PAD_ZEROES + LEFT_JUSTIFIED)))
        while (size-- > 0)
            cwrite(&str, ' ', max_characters);
    if (sign)
        cwrite(&str, sign, max_characters);
    if (type & SPECIAL)
    {
        if (base == 8 || base == 16)
            cwrite(&str, '0', max_characters);
        if (base == 16)
            cwrite(&str, (type & LOWER_CASE)?'x':'X', max_characters);
    }
    if (!(type & LEFT_JUSTIFIED))
        while (size-- > 0)
            cwrite(&str, c, max_characters);

    while (i < precision--)
        cwrite(&str, '0', max_characters);
    while (i-- > 0)
        cwrite(&str, tmp[i], max_characters);
    while (size-- > 0)
        cwrite(&str, ' ', max_characters);

    return str;
}

static int skip_atoi(const char **s)
{
    int i = 0;
    while (isdigit(**s))
        i = i * 10 + *((*s)++) - '0';
    return i;
}

int apex_vsnprintf(char* buf, int max_characters, const char *fmt, va_list args)
{
    int len;
    int i;
    int flags;
    int field_width;
    int precision;
    int *ip;
    char *str;
    char *s;

    for (str = buf; *fmt; fmt++)
    {
        if (*fmt != '%')
        {
            cwrite(&str, *fmt, &max_characters);
            continue;
        }

        flags = 0;
    get_flags:
        fmt++;
        switch (*fmt)
        {
            case '-': flags |= LEFT_JUSTIFIED; goto get_flags;
            case '+': flags |= PLUS; goto get_flags;
            case ' ': flags |= SPACE; goto get_flags;
            case '#': flags |= SPECIAL; goto get_flags;
            case '0': flags |= PAD_ZEROES; goto get_flags;
        }

        field_width = -1;
        if (isdigit(*fmt))
            field_width = skip_atoi(&fmt);
        else if (*fmt == '*')
        {
            field_width = va_arg(args, int);
            if (field_width < 0)
            {
                field_width = -field_width;
                flags |= LEFT_JUSTIFIED;
            }
        }

        precision = -1;
        if (*fmt == '.')
        {
            fmt++;
            if (isdigit(*fmt))
                precision = skip_atoi(&fmt);
            else if (*fmt == '*')
                precision = va_arg(args, int);
            precision = (precision < 0) ? 0 : precision;
        }

        switch (*fmt)
        {
            case 'c':
                if (!(flags & LEFT_JUSTIFIED))
                    while (--field_width > 0)
                        cwrite(&str, ' ', &max_characters);
                cwrite(&str, (unsigned char)va_arg(args, int), &max_characters);
                while (--field_width > 0)
                    cwrite(&str, ' ', &max_characters);
                break;

            case 's':
                s = va_arg(args, char *);
                if (!s)
                    s = (char*)"NULL";

                len = strlen(s);
                if (precision < 0)
                    precision = len;
                else if (len > precision)
                    len = precision;

                if (!(flags & LEFT_JUSTIFIED))
                    while (len < field_width--)
                        cwrite(&str, ' ', &max_characters);
                for (i = 0; i < len; i++)
                    cwrite(&str, *s++, &max_characters);
                while (len < field_width--)
                    cwrite(&str, ' ', &max_characters);
                break;

            case 'o':
                str = num_to_base(str, &max_characters, va_arg(args, unsigned long), 8, field_width, precision, flags);
                break;

            case 'p':
                if (field_width == -1)
                {
                    field_width = 8; /* 8 characters, pointers are 32 bit */
                    flags |= PAD_ZEROES;
                }
                cwrite(&str, '0', &max_characters);
                cwrite(&str, 'x', &max_characters);
                str = num_to_base(str, &max_characters, (unsigned long)va_arg(args, void *), 16, field_width, precision, flags);
                break;

            case 'x':
                flags |= LOWER_CASE;
            case 'X':
                str = num_to_base(str, &max_characters, va_arg(args, unsigned long), 16, field_width, precision, flags);
                break;

            case 'd':
            case 'i':
                flags |= SIGNED;
            case 'u':
                str = num_to_base(str, &max_characters, va_arg(args, unsigned long), 10, field_width, precision, flags);
                break;

            case 'n':
                ip = va_arg(args, int *);
                *ip = (str - buf);
                break;

            default:
                if (*fmt != '%')
                    cwrite(&str, '%', &max_characters);
                if (*fmt)
                    cwrite(&str, *fmt, &max_characters);
                else
                    fmt--;
                break;
        }
    }

    /* Write end of string */
    cwrite(&str, '\0', &max_characters);

    return str - buf;
}

int apex_snprintf(char *buf, int max_size, const char *fmt, ...)
{
    va_list args;
    int i;

    va_start(args, fmt);
    i = apex_vsnprintf(buf, max_size, fmt, args);
    va_end(args);

    return i;
}

int apex_printf(const char *fmt, ...)
{
    va_list args;
    int i;

    va_start(args, fmt);
    // don't write the last byte
    i = apex_vsnprintf(APEX_LOG_BUFFER + APEX_LOG_INDEX, APEX_LOG_SIZE - 1 - APEX_LOG_INDEX, fmt, args);
    va_end(args);

    /* Because apex_vsnprintf writes a '\0' at the end, we want the next function call to continue from then on, ignoring the last written '\0'.
    ** So subtracting 1. */
    if(i > 0) APEX_LOG_INDEX += i - 1;

    return i;
}

#endif //ifndef APU_STDIO_H