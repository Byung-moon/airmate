.class public Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;
.super Ljava/io/ByteArrayOutputStream;
.source "DNSOutgoing.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Ljavax/jmdns/impl/DNSOutgoing;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x9
    name = "MessageOutputStream"
.end annotation


# instance fields
.field private final _offset:I

.field private final _out:Ljavax/jmdns/impl/DNSOutgoing;


# direct methods
.method constructor <init>(ILjavax/jmdns/impl/DNSOutgoing;)V
    .registers 4
    .param p1, "size"    # I
    .param p2, "out"    # Ljavax/jmdns/impl/DNSOutgoing;

    .line 36
    const/4 v0, 0x0

    invoke-direct {p0, p1, p2, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;-><init>(ILjavax/jmdns/impl/DNSOutgoing;I)V

    .line 37
    return-void
.end method

.method constructor <init>(ILjavax/jmdns/impl/DNSOutgoing;I)V
    .registers 4
    .param p1, "size"    # I
    .param p2, "out"    # Ljavax/jmdns/impl/DNSOutgoing;
    .param p3, "offset"    # I

    .line 40
    invoke-direct {p0, p1}, Ljava/io/ByteArrayOutputStream;-><init>(I)V

    .line 41
    iput-object p2, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_out:Ljavax/jmdns/impl/DNSOutgoing;

    .line 42
    iput p3, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_offset:I

    .line 43
    return-void
.end method


# virtual methods
.method writeByte(I)V
    .registers 3
    .param p1, "value"    # I

    .line 46
    and-int/lit16 v0, p1, 0xff

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->write(I)V

    .line 47
    return-void
.end method

.method writeBytes(Ljava/lang/String;II)V
    .registers 6
    .param p1, "str"    # Ljava/lang/String;
    .param p2, "off"    # I
    .param p3, "len"    # I

    .line 50
    const/4 v0, 0x0

    .local v0, "i":I
    :goto_1
    if-ge v0, p3, :cond_f

    .line 51
    add-int v1, p2, v0

    invoke-virtual {p1, v1}, Ljava/lang/String;->charAt(I)C

    move-result v1

    invoke-virtual {p0, v1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 50
    add-int/lit8 v0, v0, 0x1

    goto :goto_1

    .line 53
    .end local v0    # "i":I
    :cond_f
    return-void
.end method

.method writeBytes([B)V
    .registers 4
    .param p1, "data"    # [B

    .line 56
    if-eqz p1, :cond_7

    .line 57
    const/4 v0, 0x0

    array-length v1, p1

    invoke-virtual {p0, p1, v0, v1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeBytes([BII)V

    .line 59
    :cond_7
    return-void
.end method

.method writeBytes([BII)V
    .registers 6
    .param p1, "data"    # [B
    .param p2, "off"    # I
    .param p3, "len"    # I

    .line 62
    const/4 v0, 0x0

    .local v0, "i":I
    :goto_1
    if-ge v0, p3, :cond_d

    .line 63
    add-int v1, p2, v0

    aget-byte v1, p1, v1

    invoke-virtual {p0, v1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 62
    add-int/lit8 v0, v0, 0x1

    goto :goto_1

    .line 65
    .end local v0    # "i":I
    :cond_d
    return-void
.end method

.method writeInt(I)V
    .registers 3
    .param p1, "value"    # I

    .line 73
    shr-int/lit8 v0, p1, 0x10

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 74
    invoke-virtual {p0, p1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 75
    return-void
.end method

.method writeName(Ljava/lang/String;)V
    .registers 3
    .param p1, "name"    # Ljava/lang/String;

    .line 113
    const/4 v0, 0x1

    invoke-virtual {p0, p1, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeName(Ljava/lang/String;Z)V

    .line 114
    return-void
.end method

.method writeName(Ljava/lang/String;Z)V
    .registers 11
    .param p1, "name"    # Ljava/lang/String;
    .param p2, "useCompression"    # Z

    .line 117
    move-object v0, p1

    .line 119
    .local v0, "aName":Ljava/lang/String;
    :goto_1
    const/16 v1, 0x2e

    invoke-virtual {v0, v1}, Ljava/lang/String;->indexOf(I)I

    move-result v1

    .line 120
    .local v1, "n":I
    if-gez v1, :cond_d

    .line 121
    invoke-virtual {v0}, Ljava/lang/String;->length()I

    move-result v1

    .line 123
    :cond_d
    const/4 v2, 0x0

    if-gtz v1, :cond_14

    .line 124
    invoke-virtual {p0, v2}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 125
    return-void

    .line 127
    :cond_14
    invoke-virtual {v0, v2, v1}, Ljava/lang/String;->substring(II)Ljava/lang/String;

    move-result-object v3

    .line 128
    .local v3, "label":Ljava/lang/String;
    if-eqz p2, :cond_55

    sget-boolean v4, Ljavax/jmdns/impl/DNSOutgoing;->USE_DOMAIN_NAME_COMPRESSION:Z

    if-eqz v4, :cond_55

    .line 129
    iget-object v4, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_out:Ljavax/jmdns/impl/DNSOutgoing;

    iget-object v4, v4, Ljavax/jmdns/impl/DNSOutgoing;->_names:Ljava/util/Map;

    invoke-interface {v4, v0}, Ljava/util/Map;->get(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v4

    check-cast v4, Ljava/lang/Integer;

    .line 130
    .local v4, "offset":Ljava/lang/Integer;
    if-eqz v4, :cond_3b

    .line 131
    invoke-virtual {v4}, Ljava/lang/Integer;->intValue()I

    move-result v2

    .line 132
    .local v2, "val":I
    shr-int/lit8 v5, v2, 0x8

    or-int/lit16 v5, v5, 0xc0

    invoke-virtual {p0, v5}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 133
    and-int/lit16 v5, v2, 0xff

    invoke-virtual {p0, v5}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 134
    return-void

    .line 136
    .end local v2    # "val":I
    :cond_3b
    iget-object v5, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_out:Ljavax/jmdns/impl/DNSOutgoing;

    iget-object v5, v5, Ljavax/jmdns/impl/DNSOutgoing;->_names:Ljava/util/Map;

    invoke-virtual {p0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->size()I

    move-result v6

    iget v7, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_offset:I

    add-int/2addr v6, v7

    invoke-static {v6}, Ljava/lang/Integer;->valueOf(I)Ljava/lang/Integer;

    move-result-object v6

    invoke-interface {v5, v0, v6}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 137
    invoke-virtual {v3}, Ljava/lang/String;->length()I

    move-result v5

    invoke-virtual {p0, v3, v2, v5}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeUTF(Ljava/lang/String;II)V

    .line 138
    .end local v4    # "offset":Ljava/lang/Integer;
    goto :goto_5c

    .line 139
    :cond_55
    invoke-virtual {v3}, Ljava/lang/String;->length()I

    move-result v4

    invoke-virtual {p0, v3, v2, v4}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeUTF(Ljava/lang/String;II)V

    .line 141
    :goto_5c
    invoke-virtual {v0, v1}, Ljava/lang/String;->substring(I)Ljava/lang/String;

    move-result-object v0

    .line 142
    const-string v2, "."

    invoke-virtual {v0, v2}, Ljava/lang/String;->startsWith(Ljava/lang/String;)Z

    move-result v2

    if-eqz v2, :cond_6d

    .line 143
    const/4 v2, 0x1

    invoke-virtual {v0, v2}, Ljava/lang/String;->substring(I)Ljava/lang/String;

    move-result-object v0

    .line 145
    .end local v1    # "n":I
    .end local v3    # "label":Ljava/lang/String;
    :cond_6d
    goto :goto_1
.end method

.method writeQuestion(Ljavax/jmdns/impl/DNSQuestion;)V
    .registers 3
    .param p1, "question"    # Ljavax/jmdns/impl/DNSQuestion;

    .line 149
    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSQuestion;->getName()Ljava/lang/String;

    move-result-object v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeName(Ljava/lang/String;)V

    .line 150
    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSQuestion;->getRecordType()Ljavax/jmdns/impl/constants/DNSRecordType;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSRecordType;->indexValue()I

    move-result v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 151
    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSQuestion;->getRecordClass()Ljavax/jmdns/impl/constants/DNSRecordClass;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSRecordClass;->indexValue()I

    move-result v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 152
    return-void
.end method

.method writeRecord(Ljavax/jmdns/impl/DNSRecord;J)V
    .registers 10
    .param p1, "rec"    # Ljavax/jmdns/impl/DNSRecord;
    .param p2, "now"    # J

    .line 155
    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSRecord;->getName()Ljava/lang/String;

    move-result-object v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeName(Ljava/lang/String;)V

    .line 156
    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSRecord;->getRecordType()Ljavax/jmdns/impl/constants/DNSRecordType;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSRecordType;->indexValue()I

    move-result v0

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 157
    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSRecord;->getRecordClass()Ljavax/jmdns/impl/constants/DNSRecordClass;

    move-result-object v0

    invoke-virtual {v0}, Ljavax/jmdns/impl/constants/DNSRecordClass;->indexValue()I

    move-result v0

    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSRecord;->isUnique()Z

    move-result v1

    const/4 v2, 0x0

    if-eqz v1, :cond_2d

    iget-object v1, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_out:Ljavax/jmdns/impl/DNSOutgoing;

    invoke-virtual {v1}, Ljavax/jmdns/impl/DNSOutgoing;->isMulticast()Z

    move-result v1

    if-eqz v1, :cond_2d

    const v1, 0x8000

    goto :goto_2e

    :cond_2d
    const/4 v1, 0x0

    :goto_2e
    or-int/2addr v0, v1

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 158
    const-wide/16 v0, 0x0

    cmp-long v3, p2, v0

    if-nez v3, :cond_3d

    invoke-virtual {p1}, Ljavax/jmdns/impl/DNSRecord;->getTTL()I

    move-result v0

    goto :goto_41

    :cond_3d
    invoke-virtual {p1, p2, p3}, Ljavax/jmdns/impl/DNSRecord;->getRemainingTTL(J)I

    move-result v0

    :goto_41
    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeInt(I)V

    .line 161
    new-instance v0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;

    const/16 v1, 0x200

    iget-object v3, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_out:Ljavax/jmdns/impl/DNSOutgoing;

    iget v4, p0, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->_offset:I

    invoke-virtual {p0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->size()I

    move-result v5

    add-int/2addr v4, v5

    add-int/lit8 v4, v4, 0x2

    invoke-direct {v0, v1, v3, v4}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;-><init>(ILjavax/jmdns/impl/DNSOutgoing;I)V

    .line 162
    .local v0, "record":Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;
    invoke-virtual {p1, v0}, Ljavax/jmdns/impl/DNSRecord;->write(Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;)V

    .line 163
    invoke-virtual {v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->toByteArray()[B

    move-result-object v1

    .line 165
    .local v1, "byteArray":[B
    array-length v3, v1

    invoke-virtual {p0, v3}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeShort(I)V

    .line 166
    array-length v3, v1

    invoke-virtual {p0, v1, v2, v3}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->write([BII)V

    .line 167
    return-void
.end method

.method writeShort(I)V
    .registers 3
    .param p1, "value"    # I

    .line 68
    shr-int/lit8 v0, p1, 0x8

    invoke-virtual {p0, v0}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 69
    invoke-virtual {p0, p1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 70
    return-void
.end method

.method writeUTF(Ljava/lang/String;II)V
    .registers 11
    .param p1, "str"    # Ljava/lang/String;
    .param p2, "off"    # I
    .param p3, "len"    # I

    .line 79
    const/4 v0, 0x0

    .line 80
    .local v0, "utflen":I
    const/4 v1, 0x0

    move v2, v0

    const/4 v0, 0x0

    .local v0, "i":I
    .local v2, "utflen":I
    :goto_4
    const/16 v3, 0x7ff

    const/16 v4, 0x7f

    const/4 v5, 0x1

    if-ge v0, p3, :cond_22

    .line 81
    add-int v6, p2, v0

    invoke-virtual {p1, v6}, Ljava/lang/String;->charAt(I)C

    move-result v6

    .line 82
    .local v6, "ch":I
    if-lt v6, v5, :cond_18

    if-gt v6, v4, :cond_18

    .line 83
    add-int/lit8 v2, v2, 0x1

    goto :goto_1f

    .line 85
    :cond_18
    if-le v6, v3, :cond_1d

    .line 86
    add-int/lit8 v2, v2, 0x3

    goto :goto_1f

    .line 88
    :cond_1d
    add-int/lit8 v2, v2, 0x2

    .line 80
    .end local v6    # "ch":I
    :goto_1f
    add-int/lit8 v0, v0, 0x1

    goto :goto_4

    .line 93
    .end local v0    # "i":I
    :cond_22
    invoke-virtual {p0, v2}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 95
    nop

    .local v1, "i":I
    :goto_26
    move v0, v1

    .end local v1    # "i":I
    .restart local v0    # "i":I
    if-ge v0, p3, :cond_6a

    .line 96
    add-int v1, p2, v0

    invoke-virtual {p1, v1}, Ljava/lang/String;->charAt(I)C

    move-result v1

    .line 97
    .local v1, "ch":I
    if-lt v1, v5, :cond_37

    if-gt v1, v4, :cond_37

    .line 98
    invoke-virtual {p0, v1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    goto :goto_67

    .line 100
    :cond_37
    if-le v1, v3, :cond_55

    .line 101
    shr-int/lit8 v6, v1, 0xc

    and-int/lit8 v6, v6, 0xf

    or-int/lit16 v6, v6, 0xe0

    invoke-virtual {p0, v6}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 102
    shr-int/lit8 v6, v1, 0x6

    and-int/lit8 v6, v6, 0x3f

    or-int/lit16 v6, v6, 0x80

    invoke-virtual {p0, v6}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 103
    shr-int/lit8 v6, v1, 0x0

    and-int/lit8 v6, v6, 0x3f

    or-int/lit16 v6, v6, 0x80

    invoke-virtual {p0, v6}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    goto :goto_67

    .line 105
    :cond_55
    shr-int/lit8 v6, v1, 0x6

    and-int/lit8 v6, v6, 0x1f

    or-int/lit16 v6, v6, 0xc0

    invoke-virtual {p0, v6}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 106
    shr-int/lit8 v6, v1, 0x0

    and-int/lit8 v6, v6, 0x3f

    or-int/lit16 v6, v6, 0x80

    invoke-virtual {p0, v6}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeByte(I)V

    .line 95
    .end local v1    # "ch":I
    :goto_67
    add-int/lit8 v1, v0, 0x1

    goto :goto_26

    .line 110
    .end local v0    # "i":I
    :cond_6a
    return-void
.end method
