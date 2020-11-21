.class public final enum Ljavax/jmdns/impl/constants/DNSResultCode;
.super Ljava/lang/Enum;
.source "DNSResultCode.java"


# annotations
.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Enum<",
        "Ljavax/jmdns/impl/constants/DNSResultCode;",
        ">;"
    }
.end annotation


# static fields
.field private static final synthetic $VALUES:[Ljavax/jmdns/impl/constants/DNSResultCode;

.field static final ExtendedRCode_MASK:I = 0xff

.field public static final enum FormErr:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum NXDomain:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum NXRRSet:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum NoError:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum NotAuth:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum NotImp:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum NotZone:Ljavax/jmdns/impl/constants/DNSResultCode;

.field static final RCode_MASK:I = 0xf

.field public static final enum Refused:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum ServFail:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum Unknown:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum YXDomain:Ljavax/jmdns/impl/constants/DNSResultCode;

.field public static final enum YXRRSet:Ljavax/jmdns/impl/constants/DNSResultCode;


# instance fields
.field private final _externalName:Ljava/lang/String;

.field private final _index:I


# direct methods
.method static constructor <clinit>()V
    .registers 15

    .line 15
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "Unknown"

    const-string v2, "Unknown"

    const/4 v3, 0x0

    const v4, 0xffff

    invoke-direct {v0, v1, v3, v2, v4}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->Unknown:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 19
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "NoError"

    const-string v2, "No Error"

    const/4 v4, 0x1

    invoke-direct {v0, v1, v4, v2, v3}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->NoError:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 23
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "FormErr"

    const-string v2, "Format Error"

    const/4 v5, 0x2

    invoke-direct {v0, v1, v5, v2, v4}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->FormErr:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 27
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "ServFail"

    const-string v2, "Server Failure"

    const/4 v6, 0x3

    invoke-direct {v0, v1, v6, v2, v5}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->ServFail:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 31
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "NXDomain"

    const-string v2, "Non-Existent Domain"

    const/4 v7, 0x4

    invoke-direct {v0, v1, v7, v2, v6}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->NXDomain:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 35
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "NotImp"

    const-string v2, "Not Implemented"

    const/4 v8, 0x5

    invoke-direct {v0, v1, v8, v2, v7}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->NotImp:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 39
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "Refused"

    const-string v2, "Query Refused"

    const/4 v9, 0x6

    invoke-direct {v0, v1, v9, v2, v8}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->Refused:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 43
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "YXDomain"

    const-string v2, "Name Exists when it should not"

    const/4 v10, 0x7

    invoke-direct {v0, v1, v10, v2, v9}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->YXDomain:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 47
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "YXRRSet"

    const-string v2, "RR Set Exists when it should not"

    const/16 v11, 0x8

    invoke-direct {v0, v1, v11, v2, v10}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->YXRRSet:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 51
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "NXRRSet"

    const-string v2, "RR Set that should exist does not"

    const/16 v12, 0x9

    invoke-direct {v0, v1, v12, v2, v11}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->NXRRSet:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 55
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "NotAuth"

    const-string v2, "Server Not Authoritative for zone"

    const/16 v13, 0xa

    invoke-direct {v0, v1, v13, v2, v12}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->NotAuth:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 59
    new-instance v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    const-string v1, "NotZone"

    const-string v2, "NotZone Name not contained in zone"

    const/16 v14, 0xb

    invoke-direct {v0, v1, v14, v2, v13}, Ljavax/jmdns/impl/constants/DNSResultCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->NotZone:Ljavax/jmdns/impl/constants/DNSResultCode;

    .line 11
    const/16 v0, 0xc

    new-array v0, v0, [Ljavax/jmdns/impl/constants/DNSResultCode;

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->Unknown:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v3

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->NoError:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v4

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->FormErr:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v5

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->ServFail:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v6

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->NXDomain:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v7

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->NotImp:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v8

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->Refused:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v9

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->YXDomain:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v10

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->YXRRSet:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v11

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->NXRRSet:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v12

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->NotAuth:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v13

    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->NotZone:Ljavax/jmdns/impl/constants/DNSResultCode;

    aput-object v1, v0, v14

    sput-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->$VALUES:[Ljavax/jmdns/impl/constants/DNSResultCode;

    return-void
.end method

.method private constructor <init>(Ljava/lang/String;ILjava/lang/String;I)V
    .registers 5
    .param p3, "name"    # Ljava/lang/String;
    .param p4, "index"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/lang/String;",
            "I)V"
        }
    .end annotation

    .line 101
    invoke-direct {p0, p1, p2}, Ljava/lang/Enum;-><init>(Ljava/lang/String;I)V

    .line 102
    iput-object p3, p0, Ljavax/jmdns/impl/constants/DNSResultCode;->_externalName:Ljava/lang/String;

    .line 103
    iput p4, p0, Ljavax/jmdns/impl/constants/DNSResultCode;->_index:I

    .line 104
    return-void
.end method

.method public static resultCodeForFlags(I)Ljavax/jmdns/impl/constants/DNSResultCode;
    .registers 7
    .param p0, "flags"    # I

    .line 129
    and-int/lit8 v0, p0, 0xf

    .line 130
    .local v0, "maskedIndex":I
    invoke-static {}, Ljavax/jmdns/impl/constants/DNSResultCode;->values()[Ljavax/jmdns/impl/constants/DNSResultCode;

    move-result-object v1

    array-length v2, v1

    const/4 v3, 0x0

    :goto_8
    if-ge v3, v2, :cond_14

    aget-object v4, v1, v3

    .line 131
    .local v4, "aCode":Ljavax/jmdns/impl/constants/DNSResultCode;
    iget v5, v4, Ljavax/jmdns/impl/constants/DNSResultCode;->_index:I

    if-ne v5, v0, :cond_11

    return-object v4

    .line 130
    .end local v4    # "aCode":Ljavax/jmdns/impl/constants/DNSResultCode;
    :cond_11
    add-int/lit8 v3, v3, 0x1

    goto :goto_8

    .line 133
    :cond_14
    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->Unknown:Ljavax/jmdns/impl/constants/DNSResultCode;

    return-object v1
.end method

.method public static resultCodeForFlags(II)Ljavax/jmdns/impl/constants/DNSResultCode;
    .registers 8
    .param p0, "flags"    # I
    .param p1, "extendedRCode"    # I

    .line 137
    shr-int/lit8 v0, p1, 0x1c

    and-int/lit16 v0, v0, 0xff

    and-int/lit8 v1, p0, 0xf

    or-int/2addr v0, v1

    .line 138
    .local v0, "maskedIndex":I
    invoke-static {}, Ljavax/jmdns/impl/constants/DNSResultCode;->values()[Ljavax/jmdns/impl/constants/DNSResultCode;

    move-result-object v1

    array-length v2, v1

    const/4 v3, 0x0

    :goto_d
    if-ge v3, v2, :cond_19

    aget-object v4, v1, v3

    .line 139
    .local v4, "aCode":Ljavax/jmdns/impl/constants/DNSResultCode;
    iget v5, v4, Ljavax/jmdns/impl/constants/DNSResultCode;->_index:I

    if-ne v5, v0, :cond_16

    return-object v4

    .line 138
    .end local v4    # "aCode":Ljavax/jmdns/impl/constants/DNSResultCode;
    :cond_16
    add-int/lit8 v3, v3, 0x1

    goto :goto_d

    .line 141
    :cond_19
    sget-object v1, Ljavax/jmdns/impl/constants/DNSResultCode;->Unknown:Ljavax/jmdns/impl/constants/DNSResultCode;

    return-object v1
.end method

.method public static valueOf(Ljava/lang/String;)Ljavax/jmdns/impl/constants/DNSResultCode;
    .registers 2
    .param p0, "name"    # Ljava/lang/String;

    .line 11
    const-class v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    invoke-static {v0, p0}, Ljava/lang/Enum;->valueOf(Ljava/lang/Class;Ljava/lang/String;)Ljava/lang/Enum;

    move-result-object v0

    check-cast v0, Ljavax/jmdns/impl/constants/DNSResultCode;

    return-object v0
.end method

.method public static values()[Ljavax/jmdns/impl/constants/DNSResultCode;
    .registers 1

    .line 11
    sget-object v0, Ljavax/jmdns/impl/constants/DNSResultCode;->$VALUES:[Ljavax/jmdns/impl/constants/DNSResultCode;

    invoke-virtual {v0}, [Ljavax/jmdns/impl/constants/DNSResultCode;->clone()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, [Ljavax/jmdns/impl/constants/DNSResultCode;

    return-object v0
.end method


# virtual methods
.method public externalName()Ljava/lang/String;
    .registers 2

    .line 112
    iget-object v0, p0, Ljavax/jmdns/impl/constants/DNSResultCode;->_externalName:Ljava/lang/String;

    return-object v0
.end method

.method public indexValue()I
    .registers 2

    .line 121
    iget v0, p0, Ljavax/jmdns/impl/constants/DNSResultCode;->_index:I

    return v0
.end method

.method public toString()Ljava/lang/String;
    .registers 3

    .line 146
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    invoke-virtual {p0}, Ljavax/jmdns/impl/constants/DNSResultCode;->name()Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, " index "

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {p0}, Ljavax/jmdns/impl/constants/DNSResultCode;->indexValue()I

    move-result v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method
