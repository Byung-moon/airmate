.class synthetic Ljavax/jmdns/impl/JmDNSImpl$8;
.super Ljava/lang/Object;
.source "JmDNSImpl.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Ljavax/jmdns/impl/JmDNSImpl;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1008
    name = null
.end annotation


# static fields
.field static final synthetic $SwitchMap$javax$jmdns$impl$JmDNSImpl$Operation:[I


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 1312
    invoke-static {}, Ljavax/jmdns/impl/JmDNSImpl$Operation;->values()[Ljavax/jmdns/impl/JmDNSImpl$Operation;

    move-result-object v0

    array-length v0, v0

    new-array v0, v0, [I

    sput-object v0, Ljavax/jmdns/impl/JmDNSImpl$8;->$SwitchMap$javax$jmdns$impl$JmDNSImpl$Operation:[I

    :try_start_9
    sget-object v0, Ljavax/jmdns/impl/JmDNSImpl$8;->$SwitchMap$javax$jmdns$impl$JmDNSImpl$Operation:[I

    sget-object v1, Ljavax/jmdns/impl/JmDNSImpl$Operation;->Add:Ljavax/jmdns/impl/JmDNSImpl$Operation;

    invoke-virtual {v1}, Ljavax/jmdns/impl/JmDNSImpl$Operation;->ordinal()I

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
    sget-object v0, Ljavax/jmdns/impl/JmDNSImpl$8;->$SwitchMap$javax$jmdns$impl$JmDNSImpl$Operation:[I

    sget-object v1, Ljavax/jmdns/impl/JmDNSImpl$Operation;->Remove:Ljavax/jmdns/impl/JmDNSImpl$Operation;

    invoke-virtual {v1}, Ljavax/jmdns/impl/JmDNSImpl$Operation;->ordinal()I

    move-result v1

    const/4 v2, 0x2

    aput v2, v0, v1
    :try_end_21
    .catch Ljava/lang/NoSuchFieldError; {:try_start_16 .. :try_end_21} :catch_22

    goto :goto_23

    :catch_22
    move-exception v0

    :goto_23
    return-void
.end method
