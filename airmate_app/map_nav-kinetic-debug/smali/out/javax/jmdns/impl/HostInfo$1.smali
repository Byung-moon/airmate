.class synthetic Ljavax/jmdns/impl/HostInfo$1;
.super Ljava/lang/Object;
.source "HostInfo.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Ljavax/jmdns/impl/HostInfo;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1008
    name = null
.end annotation


# static fields
.field static final synthetic $SwitchMap$javax$jmdns$impl$constants$DNSRecordType:[I


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 199
    invoke-static {}, Ljavax/jmdns/impl/constants/DNSRecordType;->values()[Ljavax/jmdns/impl/constants/DNSRecordType;

    move-result-object v0

    array-length v0, v0

    new-array v0, v0, [I

    sput-object v0, Ljavax/jmdns/impl/HostInfo$1;->$SwitchMap$javax$jmdns$impl$constants$DNSRecordType:[I

    :try_start_9
    sget-object v0, Ljavax/jmdns/impl/HostInfo$1;->$SwitchMap$javax$jmdns$impl$constants$DNSRecordType:[I

    sget-object v1, Ljavax/jmdns/impl/constants/DNSRecordType;->TYPE_A:Ljavax/jmdns/impl/constants/DNSRecordType;

    invoke-virtual {v1}, Ljavax/jmdns/impl/constants/DNSRecordType;->ordinal()I

    move-result v1

    const/4 v2, 0x1

    aput v2, v0, v1
    :try_end_14
    .catch Ljava/lang/NoSuchFieldError; {:try_start_9 .. :try_end_14} :catch_15

    goto :goto_16

    :catch_15
    move-exception v0

    :goto_16
    :try_start_16
    sget-object v0, Ljavax/jmdns/impl/HostInfo$1;->$SwitchMap$javax$jmdns$impl$constants$DNSRecordType:[I

    sget-object v1, Ljavax/jmdns/impl/constants/DNSRecordType;->TYPE_A6:Ljavax/jmdns/impl/constants/DNSRecordType;

    invoke-virtual {v1}, Ljavax/jmdns/impl/constants/DNSRecordType;->ordinal()I

    move-result v1

    const/4 v2, 0x2

    aput v2, v0, v1
    :try_end_21
    .catch Ljava/lang/NoSuchFieldError; {:try_start_16 .. :try_end_21} :catch_22

    goto :goto_23

    :catch_22
    move-exception v0

    :goto_23
    :try_start_23
    sget-object v0, Ljavax/jmdns/impl/HostInfo$1;->$SwitchMap$javax$jmdns$impl$constants$DNSRecordType:[I

    sget-object v1, Ljavax/jmdns/impl/constants/DNSRecordType;->TYPE_AAAA:Ljavax/jmdns/impl/constants/DNSRecordType;

    invoke-virtual {v1}, Ljavax/jmdns/impl/constants/DNSRecordType;->ordinal()I

    move-result v1

    const/4 v2, 0x3

    aput v2, v0, v1
    :try_end_2e
    .catch Ljava/lang/NoSuchFieldError; {:try_start_23 .. :try_end_2e} :catch_2f

    goto :goto_30

    :catch_2f
    move-exception v0

    :goto_30
    return-void
.end method
