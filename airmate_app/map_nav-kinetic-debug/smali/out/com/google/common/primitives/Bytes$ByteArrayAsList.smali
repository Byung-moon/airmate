.class Lcom/google/common/primitives/Bytes$ByteArrayAsList;
.super Ljava/util/AbstractList;
.source "Bytes.java"

# interfaces
.implements Ljava/util/RandomAccess;
.implements Ljava/io/Serializable;


# annotations
.annotation build Lcom/google/common/annotations/GwtCompatible;
.end annotation

.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/primitives/Bytes;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "ByteArrayAsList"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/util/AbstractList<",
        "Ljava/lang/Byte;",
        ">;",
        "Ljava/util/RandomAccess;",
        "Ljava/io/Serializable;"
    }
.end annotation


# static fields
.field private static final serialVersionUID:J


# instance fields
.field final array:[B

.field final end:I

.field final start:I


# direct methods
.method constructor <init>([B)V
    .registers 4
    .param p1, "array"    # [B

    .line 275
    array-length v0, p1

    const/4 v1, 0x0

    invoke-direct {p0, p1, v1, v0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;-><init>([BII)V

    .line 276
    return-void
.end method

.method constructor <init>([BII)V
    .registers 4
    .param p1, "array"    # [B
    .param p2, "start"    # I
    .param p3, "end"    # I

    .line 278
    invoke-direct {p0}, Ljava/util/AbstractList;-><init>()V

    .line 279
    iput-object p1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    .line 280
    iput p2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    .line 281
    iput p3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    .line 282
    return-void
.end method


# virtual methods
.method public contains(Ljava/lang/Object;)Z
    .registers 6
    .param p1, "target"    # Ljava/lang/Object;

    .line 299
    instance-of v0, p1, Ljava/lang/Byte;

    if-eqz v0, :cond_1a

    iget-object v0, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    move-object v1, p1

    check-cast v1, Ljava/lang/Byte;

    invoke-virtual {v1}, Ljava/lang/Byte;->byteValue()B

    move-result v1

    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    iget v3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    invoke-static {v0, v1, v2, v3}, Lcom/google/common/primitives/Bytes;->access$000([BBII)I

    move-result v0

    const/4 v1, -0x1

    if-eq v0, v1, :cond_1a

    const/4 v0, 0x1

    goto :goto_1b

    :cond_1a
    const/4 v0, 0x0

    :goto_1b
    return v0
.end method

.method public equals(Ljava/lang/Object;)Z
    .registers 10
    .param p1, "object"    # Ljava/lang/Object;

    .line 342
    const/4 v0, 0x1

    if-ne p1, p0, :cond_4

    .line 343
    return v0

    .line 345
    :cond_4
    instance-of v1, p1, Lcom/google/common/primitives/Bytes$ByteArrayAsList;

    if-eqz v1, :cond_2f

    .line 346
    move-object v1, p1

    check-cast v1, Lcom/google/common/primitives/Bytes$ByteArrayAsList;

    .line 347
    .local v1, "that":Lcom/google/common/primitives/Bytes$ByteArrayAsList;
    invoke-virtual {p0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v2

    .line 348
    .local v2, "size":I
    invoke-virtual {v1}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v3

    const/4 v4, 0x0

    if-eq v3, v2, :cond_17

    .line 349
    return v4

    .line 351
    :cond_17
    const/4 v3, 0x0

    .local v3, "i":I
    :goto_18
    if-ge v3, v2, :cond_2e

    .line 352
    iget-object v5, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v6, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v6, v3

    aget-byte v5, v5, v6

    iget-object v6, v1, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v7, v1, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v7, v3

    aget-byte v6, v6, v7

    if-eq v5, v6, :cond_2b

    .line 353
    return v4

    .line 351
    :cond_2b
    add-int/lit8 v3, v3, 0x1

    goto :goto_18

    .line 356
    .end local v3    # "i":I
    :cond_2e
    return v0

    .line 358
    .end local v1    # "that":Lcom/google/common/primitives/Bytes$ByteArrayAsList;
    .end local v2    # "size":I
    :cond_2f
    invoke-super {p0, p1}, Ljava/util/AbstractList;->equals(Ljava/lang/Object;)Z

    move-result v0

    return v0
.end method

.method public get(I)Ljava/lang/Byte;
    .registers 4
    .param p1, "index"    # I

    .line 293
    invoke-virtual {p0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v0

    invoke-static {p1, v0}, Lcom/google/common/base/Preconditions;->checkElementIndex(II)I

    .line 294
    iget-object v0, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v1, p1

    aget-byte v0, v0, v1

    invoke-static {v0}, Ljava/lang/Byte;->valueOf(B)Ljava/lang/Byte;

    move-result-object v0

    return-object v0
.end method

.method public bridge synthetic get(I)Ljava/lang/Object;
    .registers 3
    .param p1, "x0"    # I

    .line 267
    invoke-virtual {p0, p1}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->get(I)Ljava/lang/Byte;

    move-result-object v0

    return-object v0
.end method

.method public hashCode()I
    .registers 5

    .line 362
    const/4 v0, 0x1

    .line 363
    .local v0, "result":I
    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    .local v1, "i":I
    :goto_3
    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    if-ge v1, v2, :cond_16

    .line 364
    mul-int/lit8 v2, v0, 0x1f

    iget-object v3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    aget-byte v3, v3, v1

    invoke-static {v3}, Lcom/google/common/primitives/Bytes;->hashCode(B)I

    move-result v3

    add-int v0, v2, v3

    .line 363
    add-int/lit8 v1, v1, 0x1

    goto :goto_3

    .line 366
    .end local v1    # "i":I
    :cond_16
    return v0
.end method

.method public indexOf(Ljava/lang/Object;)I
    .registers 6
    .param p1, "target"    # Ljava/lang/Object;

    .line 305
    instance-of v0, p1, Ljava/lang/Byte;

    if-eqz v0, :cond_1c

    .line 306
    iget-object v0, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    move-object v1, p1

    check-cast v1, Ljava/lang/Byte;

    invoke-virtual {v1}, Ljava/lang/Byte;->byteValue()B

    move-result v1

    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    iget v3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    invoke-static {v0, v1, v2, v3}, Lcom/google/common/primitives/Bytes;->access$000([BBII)I

    move-result v0

    .line 307
    .local v0, "i":I
    if-ltz v0, :cond_1c

    .line 308
    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    sub-int v1, v0, v1

    return v1

    .line 311
    .end local v0    # "i":I
    :cond_1c
    const/4 v0, -0x1

    return v0
.end method

.method public isEmpty()Z
    .registers 2

    .line 289
    const/4 v0, 0x0

    return v0
.end method

.method public lastIndexOf(Ljava/lang/Object;)I
    .registers 6
    .param p1, "target"    # Ljava/lang/Object;

    .line 316
    instance-of v0, p1, Ljava/lang/Byte;

    if-eqz v0, :cond_1c

    .line 317
    iget-object v0, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    move-object v1, p1

    check-cast v1, Ljava/lang/Byte;

    invoke-virtual {v1}, Ljava/lang/Byte;->byteValue()B

    move-result v1

    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    iget v3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    invoke-static {v0, v1, v2, v3}, Lcom/google/common/primitives/Bytes;->access$100([BBII)I

    move-result v0

    .line 318
    .local v0, "i":I
    if-ltz v0, :cond_1c

    .line 319
    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    sub-int v1, v0, v1

    return v1

    .line 322
    .end local v0    # "i":I
    :cond_1c
    const/4 v0, -0x1

    return v0
.end method

.method public set(ILjava/lang/Byte;)Ljava/lang/Byte;
    .registers 7
    .param p1, "index"    # I
    .param p2, "element"    # Ljava/lang/Byte;

    .line 326
    invoke-virtual {p0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v0

    invoke-static {p1, v0}, Lcom/google/common/base/Preconditions;->checkElementIndex(II)I

    .line 327
    iget-object v0, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v1, p1

    aget-byte v0, v0, v1

    .line 328
    .local v0, "oldValue":B
    iget-object v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v2, p1

    invoke-static {p2}, Lcom/google/common/base/Preconditions;->checkNotNull(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v3

    check-cast v3, Ljava/lang/Byte;

    invoke-virtual {v3}, Ljava/lang/Byte;->byteValue()B

    move-result v3

    aput-byte v3, v1, v2

    .line 329
    invoke-static {v0}, Ljava/lang/Byte;->valueOf(B)Ljava/lang/Byte;

    move-result-object v1

    return-object v1
.end method

.method public bridge synthetic set(ILjava/lang/Object;)Ljava/lang/Object;
    .registers 4
    .param p1, "x0"    # I
    .param p2, "x1"    # Ljava/lang/Object;

    .line 267
    move-object v0, p2

    check-cast v0, Ljava/lang/Byte;

    invoke-virtual {p0, p1, v0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->set(ILjava/lang/Byte;)Ljava/lang/Byte;

    move-result-object v0

    return-object v0
.end method

.method public size()I
    .registers 3

    .line 285
    iget v0, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    sub-int/2addr v0, v1

    return v0
.end method

.method public subList(II)Ljava/util/List;
    .registers 8
    .param p1, "fromIndex"    # I
    .param p2, "toIndex"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(II)",
            "Ljava/util/List<",
            "Ljava/lang/Byte;",
            ">;"
        }
    .end annotation

    .line 333
    invoke-virtual {p0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v0

    .line 334
    .local v0, "size":I
    invoke-static {p1, p2, v0}, Lcom/google/common/base/Preconditions;->checkPositionIndexes(III)V

    .line 335
    if-ne p1, p2, :cond_e

    .line 336
    invoke-static {}, Ljava/util/Collections;->emptyList()Ljava/util/List;

    move-result-object v1

    return-object v1

    .line 338
    :cond_e
    new-instance v1, Lcom/google/common/primitives/Bytes$ByteArrayAsList;

    iget-object v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v3, p1

    iget v4, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    add-int/2addr v4, p2

    invoke-direct {v1, v2, v3, v4}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;-><init>([BII)V

    return-object v1
.end method

.method toByteArray()[B
    .registers 6

    .line 380
    invoke-virtual {p0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v0

    .line 381
    .local v0, "size":I
    new-array v1, v0, [B

    .line 382
    .local v1, "result":[B
    iget-object v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v3, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    const/4 v4, 0x0

    invoke-static {v2, v3, v1, v4, v0}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 383
    return-object v1
.end method

.method public toString()Ljava/lang/String;
    .registers 4

    .line 370
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-virtual {p0}, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->size()I

    move-result v1

    mul-int/lit8 v1, v1, 0x5

    invoke-direct {v0, v1}, Ljava/lang/StringBuilder;-><init>(I)V

    .line 371
    .local v0, "builder":Ljava/lang/StringBuilder;
    const/16 v1, 0x5b

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(C)Ljava/lang/StringBuilder;

    iget-object v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    aget-byte v1, v1, v2

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    .line 372
    iget v1, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->start:I

    .local v1, "i":I
    :goto_1b
    add-int/lit8 v1, v1, 0x1

    iget v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->end:I

    if-ge v1, v2, :cond_2e

    .line 373
    const-string v2, ", "

    invoke-virtual {v0, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    iget-object v2, p0, Lcom/google/common/primitives/Bytes$ByteArrayAsList;->array:[B

    aget-byte v2, v2, v1

    invoke-virtual {v0, v2}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    .line 372
    goto :goto_1b

    .line 375
    .end local v1    # "i":I
    :cond_2e
    const/16 v1, 0x5d

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(C)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v1

    return-object v1
.end method
