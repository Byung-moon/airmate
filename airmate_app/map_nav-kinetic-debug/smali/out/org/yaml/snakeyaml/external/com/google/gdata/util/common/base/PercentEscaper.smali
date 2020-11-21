.class public Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;
.super Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/UnicodeEscaper;
.source "PercentEscaper.java"


# static fields
.field public static final SAFECHARS_URLENCODER:Ljava/lang/String; = "-_.*"

.field public static final SAFEPATHCHARS_URLENCODER:Ljava/lang/String; = "-_.!~*\'()@:$&,;="

.field public static final SAFEQUERYSTRINGCHARS_URLENCODER:Ljava/lang/String; = "-_.!~*\'()@:$,;/?:"

.field private static final UPPER_HEX_DIGITS:[C

.field private static final URI_ESCAPED_SPACE:[C


# instance fields
.field private final plusForSpace:Z

.field private final safeOctets:[Z


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 90
    const/4 v0, 0x1

    new-array v0, v0, [C

    const/4 v1, 0x0

    const/16 v2, 0x2b

    aput-char v2, v0, v1

    sput-object v0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->URI_ESCAPED_SPACE:[C

    .line 92
    const-string v0, "0123456789ABCDEF"

    invoke-virtual {v0}, Ljava/lang/String;->toCharArray()[C

    move-result-object v0

    sput-object v0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    return-void
.end method

.method public constructor <init>(Ljava/lang/String;Z)V
    .registers 5
    .param p1, "safeChars"    # Ljava/lang/String;
    .param p2, "plusForSpace"    # Z

    .line 120
    invoke-direct {p0}, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/UnicodeEscaper;-><init>()V

    .line 122
    const-string v0, ".*[0-9A-Za-z].*"

    invoke-virtual {p1, v0}, Ljava/lang/String;->matches(Ljava/lang/String;)Z

    move-result v0

    if-nez v0, :cond_37

    .line 129
    if-eqz p2, :cond_1e

    const-string v0, " "

    invoke-virtual {p1, v0}, Ljava/lang/String;->contains(Ljava/lang/CharSequence;)Z

    move-result v0

    if-nez v0, :cond_16

    goto :goto_1e

    .line 130
    :cond_16
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "plusForSpace cannot be specified when space is a \'safe\' character"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 133
    :cond_1e
    :goto_1e
    const-string v0, "%"

    invoke-virtual {p1, v0}, Ljava/lang/String;->contains(Ljava/lang/CharSequence;)Z

    move-result v0

    if-nez v0, :cond_2f

    .line 136
    iput-boolean p2, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->plusForSpace:Z

    .line 137
    invoke-static {p1}, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->createSafeOctets(Ljava/lang/String;)[Z

    move-result-object v0

    iput-object v0, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    .line 138
    return-void

    .line 134
    :cond_2f
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "The \'%\' character cannot be specified as \'safe\'"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 123
    :cond_37
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "Alphanumeric characters are always \'safe\' and should not be explicitly specified"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method private static createSafeOctets(Ljava/lang/String;)[Z
    .registers 9
    .param p0, "safeChars"    # Ljava/lang/String;

    .line 146
    const/16 v0, 0x7a

    .line 147
    .local v0, "maxChar":I
    invoke-virtual {p0}, Ljava/lang/String;->toCharArray()[C

    move-result-object v1

    .line 148
    .local v1, "safeCharArray":[C
    move-object v2, v1

    .local v2, "arr$":[C
    array-length v3, v2

    .local v3, "len$":I
    const/4 v4, 0x0

    move v5, v0

    const/4 v0, 0x0

    .local v0, "i$":I
    .local v5, "maxChar":I
    :goto_b
    if-ge v0, v3, :cond_16

    aget-char v6, v2, v0

    .line 149
    .local v6, "c":C
    invoke-static {v6, v5}, Ljava/lang/Math;->max(II)I

    move-result v5

    .line 148
    .end local v6    # "c":C
    add-int/lit8 v0, v0, 0x1

    goto :goto_b

    .line 151
    .end local v0    # "i$":I
    .end local v2    # "arr$":[C
    .end local v3    # "len$":I
    :cond_16
    add-int/lit8 v0, v5, 0x1

    new-array v0, v0, [Z

    .line 152
    .local v0, "octets":[Z
    const/16 v2, 0x30

    .local v2, "c":I
    :goto_1c
    const/16 v3, 0x39

    const/4 v6, 0x1

    if-gt v2, v3, :cond_26

    .line 153
    aput-boolean v6, v0, v2

    .line 152
    add-int/lit8 v2, v2, 0x1

    goto :goto_1c

    .line 155
    .end local v2    # "c":I
    :cond_26
    const/16 v2, 0x41

    .restart local v2    # "c":I
    :goto_28
    const/16 v3, 0x5a

    if-gt v2, v3, :cond_31

    .line 156
    aput-boolean v6, v0, v2

    .line 155
    add-int/lit8 v2, v2, 0x1

    goto :goto_28

    .line 158
    .end local v2    # "c":I
    :cond_31
    const/16 v2, 0x61

    .restart local v2    # "c":I
    :goto_33
    const/16 v3, 0x7a

    if-gt v2, v3, :cond_3c

    .line 159
    aput-boolean v6, v0, v2

    .line 158
    add-int/lit8 v2, v2, 0x1

    goto :goto_33

    .line 161
    .end local v2    # "c":I
    :cond_3c
    move-object v2, v1

    .local v2, "arr$":[C
    array-length v3, v2

    .restart local v3    # "len$":I
    .local v4, "i$":I
    :goto_3e
    if-ge v4, v3, :cond_47

    aget-char v7, v2, v4

    .line 162
    .local v7, "c":C
    aput-boolean v6, v0, v7

    .line 161
    .end local v7    # "c":C
    add-int/lit8 v4, v4, 0x1

    goto :goto_3e

    .line 164
    .end local v2    # "arr$":[C
    .end local v3    # "len$":I
    .end local v4    # "i$":I
    :cond_47
    return-object v0
.end method


# virtual methods
.method public escape(Ljava/lang/String;)Ljava/lang/String;
    .registers 6
    .param p1, "s"    # Ljava/lang/String;

    .line 190
    invoke-virtual {p1}, Ljava/lang/String;->length()I

    move-result v0

    .line 191
    .local v0, "slen":I
    const/4 v1, 0x0

    .local v1, "index":I
    :goto_5
    if-ge v1, v0, :cond_1f

    .line 192
    invoke-virtual {p1, v1}, Ljava/lang/String;->charAt(I)C

    move-result v2

    .line 193
    .local v2, "c":C
    iget-object v3, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    array-length v3, v3

    if-ge v2, v3, :cond_1a

    iget-object v3, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    aget-boolean v3, v3, v2

    if-nez v3, :cond_17

    goto :goto_1a

    .line 191
    .end local v2    # "c":C
    :cond_17
    add-int/lit8 v1, v1, 0x1

    goto :goto_5

    .line 194
    .restart local v2    # "c":C
    :cond_1a
    :goto_1a
    invoke-virtual {p0, p1, v1}, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->escapeSlow(Ljava/lang/String;I)Ljava/lang/String;

    move-result-object v3

    return-object v3

    .line 197
    .end local v1    # "index":I
    .end local v2    # "c":C
    :cond_1f
    return-object p1
.end method

.method protected escape(I)[C
    .registers 14
    .param p1, "cp"    # I

    .line 208
    iget-object v0, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    array-length v0, v0

    if-ge p1, v0, :cond_d

    iget-object v0, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    aget-boolean v0, v0, p1

    if-eqz v0, :cond_d

    .line 209
    const/4 v0, 0x0

    return-object v0

    .line 210
    :cond_d
    const/16 v0, 0x20

    if-ne p1, v0, :cond_18

    iget-boolean v0, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->plusForSpace:Z

    if-eqz v0, :cond_18

    .line 211
    sget-object v0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->URI_ESCAPED_SPACE:[C

    return-object v0

    .line 212
    :cond_18
    const/16 v0, 0x7f

    const/4 v1, 0x1

    const/4 v2, 0x0

    const/4 v3, 0x2

    const/16 v4, 0x25

    const/4 v5, 0x3

    if-gt p1, v0, :cond_37

    .line 215
    new-array v0, v5, [C

    .line 216
    .local v0, "dest":[C
    aput-char v4, v0, v2

    .line 217
    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v4, p1, 0xf

    aget-char v2, v2, v4

    aput-char v2, v0, v3

    .line 218
    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    ushr-int/lit8 v3, p1, 0x4

    aget-char v2, v2, v3

    aput-char v2, v0, v1

    .line 219
    return-object v0

    .line 220
    .end local v0    # "dest":[C
    :cond_37
    const/16 v0, 0x7ff

    const/4 v6, 0x5

    const/4 v7, 0x6

    const/16 v8, 0x8

    const/4 v9, 0x4

    if-gt p1, v0, :cond_6b

    .line 223
    new-array v0, v7, [C

    .line 224
    .restart local v0    # "dest":[C
    aput-char v4, v0, v2

    .line 225
    aput-char v4, v0, v5

    .line 226
    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v4, p1, 0xf

    aget-char v2, v2, v4

    aput-char v2, v0, v6

    .line 227
    ushr-int/2addr p1, v9

    .line 228
    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v4, p1, 0x3

    or-int/2addr v4, v8

    aget-char v2, v2, v4

    aput-char v2, v0, v9

    .line 229
    ushr-int/2addr p1, v3

    .line 230
    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v4, p1, 0xf

    aget-char v2, v2, v4

    aput-char v2, v0, v3

    .line 231
    ushr-int/2addr p1, v9

    .line 232
    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    or-int/lit8 v3, p1, 0xc

    aget-char v2, v2, v3

    aput-char v2, v0, v1

    .line 233
    return-object v0

    .line 234
    .end local v0    # "dest":[C
    :cond_6b
    const v0, 0xffff

    const/16 v10, 0x9

    const/4 v11, 0x7

    if-gt p1, v0, :cond_ac

    .line 237
    new-array v0, v10, [C

    .line 238
    .restart local v0    # "dest":[C
    aput-char v4, v0, v2

    .line 239
    const/16 v2, 0x45

    aput-char v2, v0, v1

    .line 240
    aput-char v4, v0, v5

    .line 241
    aput-char v4, v0, v7

    .line 242
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0xf

    aget-char v1, v1, v2

    aput-char v1, v0, v8

    .line 243
    ushr-int/2addr p1, v9

    .line 244
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0x3

    or-int/2addr v2, v8

    aget-char v1, v1, v2

    aput-char v1, v0, v11

    .line 245
    ushr-int/2addr p1, v3

    .line 246
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0xf

    aget-char v1, v1, v2

    aput-char v1, v0, v6

    .line 247
    ushr-int/2addr p1, v9

    .line 248
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0x3

    or-int/2addr v2, v8

    aget-char v1, v1, v2

    aput-char v1, v0, v9

    .line 249
    ushr-int/2addr p1, v3

    .line 250
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    aget-char v1, v1, p1

    aput-char v1, v0, v3

    .line 251
    return-object v0

    .line 252
    .end local v0    # "dest":[C
    :cond_ac
    const v0, 0x10ffff

    if-gt p1, v0, :cond_107

    .line 253
    const/16 v0, 0xc

    new-array v0, v0, [C

    .line 256
    .restart local v0    # "dest":[C
    aput-char v4, v0, v2

    .line 257
    const/16 v2, 0x46

    aput-char v2, v0, v1

    .line 258
    aput-char v4, v0, v5

    .line 259
    aput-char v4, v0, v7

    .line 260
    aput-char v4, v0, v10

    .line 261
    const/16 v1, 0xb

    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v4, p1, 0xf

    aget-char v2, v2, v4

    aput-char v2, v0, v1

    .line 262
    ushr-int/2addr p1, v9

    .line 263
    const/16 v1, 0xa

    sget-object v2, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v4, p1, 0x3

    or-int/2addr v4, v8

    aget-char v2, v2, v4

    aput-char v2, v0, v1

    .line 264
    ushr-int/2addr p1, v3

    .line 265
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0xf

    aget-char v1, v1, v2

    aput-char v1, v0, v8

    .line 266
    ushr-int/2addr p1, v9

    .line 267
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0x3

    or-int/2addr v2, v8

    aget-char v1, v1, v2

    aput-char v1, v0, v11

    .line 268
    ushr-int/2addr p1, v3

    .line 269
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0xf

    aget-char v1, v1, v2

    aput-char v1, v0, v6

    .line 270
    ushr-int/2addr p1, v9

    .line 271
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0x3

    or-int/2addr v2, v8

    aget-char v1, v1, v2

    aput-char v1, v0, v9

    .line 272
    ushr-int/2addr p1, v3

    .line 273
    sget-object v1, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->UPPER_HEX_DIGITS:[C

    and-int/lit8 v2, p1, 0x7

    aget-char v1, v1, v2

    aput-char v1, v0, v3

    .line 274
    return-object v0

    .line 278
    .end local v0    # "dest":[C
    :cond_107
    new-instance v0, Ljava/lang/IllegalArgumentException;

    new-instance v1, Ljava/lang/StringBuilder;

    invoke-direct {v1}, Ljava/lang/StringBuilder;-><init>()V

    const-string v2, "Invalid unicode character value "

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v1, p1}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    invoke-virtual {v1}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v1

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method protected nextEscapeIndex(Ljava/lang/CharSequence;II)I
    .registers 6
    .param p1, "csq"    # Ljava/lang/CharSequence;
    .param p2, "index"    # I
    .param p3, "end"    # I

    .line 174
    :goto_0
    if-ge p2, p3, :cond_15

    .line 175
    invoke-interface {p1, p2}, Ljava/lang/CharSequence;->charAt(I)C

    move-result v0

    .line 176
    .local v0, "c":C
    iget-object v1, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    array-length v1, v1

    if-ge v0, v1, :cond_15

    iget-object v1, p0, Lorg/yaml/snakeyaml/external/com/google/gdata/util/common/base/PercentEscaper;->safeOctets:[Z

    aget-boolean v1, v1, v0

    if-nez v1, :cond_12

    .line 177
    goto :goto_15

    .line 174
    .end local v0    # "c":C
    :cond_12
    add-int/lit8 p2, p2, 0x1

    goto :goto_0

    .line 180
    :cond_15
    :goto_15
    return p2
.end method
