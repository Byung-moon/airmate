.class public final enum Ljavax/jmdns/impl/constants/DNSOptionCode;
.super Ljava/lang/Enum;
.source "DNSOptionCode.java"


# annotations
.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Enum<",
        "Ljavax/jmdns/impl/constants/DNSOptionCode;",
        ">;"
    }
.end annotation


# static fields
.field private static final synthetic $VALUES:[Ljavax/jmdns/impl/constants/DNSOptionCode;

.field public static final enum LLQ:Ljavax/jmdns/impl/constants/DNSOptionCode;

.field public static final enum NSID:Ljavax/jmdns/impl/constants/DNSOptionCode;

.field public static final enum Owner:Ljavax/jmdns/impl/constants/DNSOptionCode;

.field public static final enum UL:Ljavax/jmdns/impl/constants/DNSOptionCode;

.field public static final enum Unknown:Ljavax/jmdns/impl/constants/DNSOptionCode;


# instance fields
.field private final _externalName:Ljava/lang/String;

.field private final _index:I


# direct methods
.method static constructor <clinit>()V
    .registers 8

    .line 16
    new-instance v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    const-string v1, "Unknown"

    const-string v2, "Unknown"

    const/4 v3, 0x0

    const v4, 0xffff

    invoke-direct {v0, v1, v3, v2, v4}, Ljavax/jmdns/impl/constants/DNSOptionCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->Unknown:Ljavax/jmdns/impl/constants/DNSOptionCode;

    .line 20
    new-instance v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    const-string v1, "LLQ"

    const-string v2, "LLQ"

    const/4 v4, 0x1

    invoke-direct {v0, v1, v4, v2, v4}, Ljavax/jmdns/impl/constants/DNSOptionCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->LLQ:Ljavax/jmdns/impl/constants/DNSOptionCode;

    .line 24
    new-instance v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    const-string v1, "UL"

    const-string v2, "UL"

    const/4 v5, 0x2

    invoke-direct {v0, v1, v5, v2, v5}, Ljavax/jmdns/impl/constants/DNSOptionCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->UL:Ljavax/jmdns/impl/constants/DNSOptionCode;

    .line 28
    new-instance v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    const-string v1, "NSID"

    const-string v2, "NSID"

    const/4 v6, 0x3

    invoke-direct {v0, v1, v6, v2, v6}, Ljavax/jmdns/impl/constants/DNSOptionCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->NSID:Ljavax/jmdns/impl/constants/DNSOptionCode;

    .line 32
    new-instance v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    const-string v1, "Owner"

    const-string v2, "Owner"

    const/4 v7, 0x4

    invoke-direct {v0, v1, v7, v2, v7}, Ljavax/jmdns/impl/constants/DNSOptionCode;-><init>(Ljava/lang/String;ILjava/lang/String;I)V

    sput-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->Owner:Ljavax/jmdns/impl/constants/DNSOptionCode;

    .line 11
    const/4 v0, 0x5

    new-array v0, v0, [Ljavax/jmdns/impl/constants/DNSOptionCode;

    sget-object v1, Ljavax/jmdns/impl/constants/DNSOptionCode;->Unknown:Ljavax/jmdns/impl/constants/DNSOptionCode;

    aput-object v1, v0, v3

    sget-object v1, Ljavax/jmdns/impl/constants/DNSOptionCode;->LLQ:Ljavax/jmdns/impl/constants/DNSOptionCode;

    aput-object v1, v0, v4

    sget-object v1, Ljavax/jmdns/impl/constants/DNSOptionCode;->UL:Ljavax/jmdns/impl/constants/DNSOptionCode;

    aput-object v1, v0, v5

    sget-object v1, Ljavax/jmdns/impl/constants/DNSOptionCode;->NSID:Ljavax/jmdns/impl/constants/DNSOptionCode;

    aput-object v1, v0, v6

    sget-object v1, Ljavax/jmdns/impl/constants/DNSOptionCode;->Owner:Ljavax/jmdns/impl/constants/DNSOptionCode;

    aput-object v1, v0, v7

    sput-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->$VALUES:[Ljavax/jmdns/impl/constants/DNSOptionCode;

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

    .line 38
    invoke-direct {p0, p1, p2}, Ljava/lang/Enum;-><init>(Ljava/lang/String;I)V

    .line 39
    iput-object p3, p0, Ljavax/jmdns/impl/constants/DNSOptionCode;->_externalName:Ljava/lang/String;

    .line 40
    iput p4, p0, Ljavax/jmdns/impl/constants/DNSOptionCode;->_index:I

    .line 41
    return-void
.end method

.method public static resultCodeForFlags(I)Ljavax/jmdns/impl/constants/DNSOptionCode;
    .registers 7
    .param p0, "optioncode"    # I

    .line 66
    move v0, p0

    .line 67
    .local v0, "maskedIndex":I
    invoke-static {}, Ljavax/jmdns/impl/constants/DNSOptionCode;->values()[Ljavax/jmdns/impl/constants/DNSOptionCode;

    move-result-object v1

    array-length v2, v1

    const/4 v3, 0x0

    :goto_7
    if-ge v3, v2, :cond_13

    aget-object v4, v1, v3

    .line 68
    .local v4, "aCode":Ljavax/jmdns/impl/constants/DNSOptionCode;
    iget v5, v4, Ljavax/jmdns/impl/constants/DNSOptionCode;->_index:I

    if-ne v5, v0, :cond_10

    return-object v4

    .line 67
    .end local v4    # "aCode":Ljavax/jmdns/impl/constants/DNSOptionCode;
    :cond_10
    add-int/lit8 v3, v3, 0x1

    goto :goto_7

    .line 70
    :cond_13
    sget-object v1, Ljavax/jmdns/impl/constants/DNSOptionCode;->Unknown:Ljavax/jmdns/impl/constants/DNSOptionCode;

    return-object v1
.end method

.method public static valueOf(Ljava/lang/String;)Ljavax/jmdns/impl/constants/DNSOptionCode;
    .registers 2
    .param p0, "name"    # Ljava/lang/String;

    .line 11
    const-class v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    invoke-static {v0, p0}, Ljava/lang/Enum;->valueOf(Ljava/lang/Class;Ljava/lang/String;)Ljava/lang/Enum;

    move-result-object v0

    check-cast v0, Ljavax/jmdns/impl/constants/DNSOptionCode;

    return-object v0
.end method

.method public static values()[Ljavax/jmdns/impl/constants/DNSOptionCode;
    .registers 1

    .line 11
    sget-object v0, Ljavax/jmdns/impl/constants/DNSOptionCode;->$VALUES:[Ljavax/jmdns/impl/constants/DNSOptionCode;

    invoke-virtual {v0}, [Ljavax/jmdns/impl/constants/DNSOptionCode;->clone()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, [Ljavax/jmdns/impl/constants/DNSOptionCode;

    return-object v0
.end method


# virtual methods
.method public externalName()Ljava/lang/String;
    .registers 2

    .line 49
    iget-object v0, p0, Ljavax/jmdns/impl/constants/DNSOptionCode;->_externalName:Ljava/lang/String;

    return-object v0
.end method

.method public indexValue()I
    .registers 2

    .line 58
    iget v0, p0, Ljavax/jmdns/impl/constants/DNSOptionCode;->_index:I

    return v0
.end method

.method public toString()Ljava/lang/String;
    .registers 3

    .line 75
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    invoke-virtual {p0}, Ljavax/jmdns/impl/constants/DNSOptionCode;->name()Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, " index "

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {p0}, Ljavax/jmdns/impl/constants/DNSOptionCode;->indexValue()I

    move-result v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method
