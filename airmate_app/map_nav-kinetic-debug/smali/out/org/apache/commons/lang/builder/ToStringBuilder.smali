.class public Lorg/apache/commons/lang/builder/ToStringBuilder;
.super Ljava/lang/Object;
.source "ToStringBuilder.java"


# static fields
.field private static defaultStyle:Lorg/apache/commons/lang/builder/ToStringStyle;


# instance fields
.field private final buffer:Ljava/lang/StringBuffer;

.field private final object:Ljava/lang/Object;

.field private final style:Lorg/apache/commons/lang/builder/ToStringStyle;


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 98
    sget-object v0, Lorg/apache/commons/lang/builder/ToStringStyle;->DEFAULT_STYLE:Lorg/apache/commons/lang/builder/ToStringStyle;

    sput-object v0, Lorg/apache/commons/lang/builder/ToStringBuilder;->defaultStyle:Lorg/apache/commons/lang/builder/ToStringStyle;

    return-void
.end method

.method public constructor <init>(Ljava/lang/Object;)V
    .registers 4
    .param p1, "object"    # Ljava/lang/Object;

    .line 212
    invoke-static {}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getDefaultStyle()Lorg/apache/commons/lang/builder/ToStringStyle;

    move-result-object v0

    const/4 v1, 0x0

    invoke-direct {p0, p1, v0, v1}, Lorg/apache/commons/lang/builder/ToStringBuilder;-><init>(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;Ljava/lang/StringBuffer;)V

    .line 213
    return-void
.end method

.method public constructor <init>(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;)V
    .registers 4
    .param p1, "object"    # Ljava/lang/Object;
    .param p2, "style"    # Lorg/apache/commons/lang/builder/ToStringStyle;

    .line 228
    const/4 v0, 0x0

    invoke-direct {p0, p1, p2, v0}, Lorg/apache/commons/lang/builder/ToStringBuilder;-><init>(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;Ljava/lang/StringBuffer;)V

    .line 229
    return-void
.end method

.method public constructor <init>(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;Ljava/lang/StringBuffer;)V
    .registers 6
    .param p1, "object"    # Ljava/lang/Object;
    .param p2, "style"    # Lorg/apache/commons/lang/builder/ToStringStyle;
    .param p3, "buffer"    # Ljava/lang/StringBuffer;

    .line 244
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 245
    if-nez p2, :cond_9

    .line 246
    invoke-static {}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getDefaultStyle()Lorg/apache/commons/lang/builder/ToStringStyle;

    move-result-object p2

    .line 248
    :cond_9
    if-nez p3, :cond_13

    .line 249
    new-instance v0, Ljava/lang/StringBuffer;

    const/16 v1, 0x200

    invoke-direct {v0, v1}, Ljava/lang/StringBuffer;-><init>(I)V

    move-object p3, v0

    .line 251
    :cond_13
    iput-object p3, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    .line 252
    iput-object p2, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    .line 253
    iput-object p1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->object:Ljava/lang/Object;

    .line 255
    invoke-virtual {p2, p3, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->appendStart(Ljava/lang/StringBuffer;Ljava/lang/Object;)V

    .line 256
    return-void
.end method

.method public static getDefaultStyle()Lorg/apache/commons/lang/builder/ToStringStyle;
    .registers 1

    .line 115
    sget-object v0, Lorg/apache/commons/lang/builder/ToStringBuilder;->defaultStyle:Lorg/apache/commons/lang/builder/ToStringStyle;

    return-object v0
.end method

.method public static reflectionToString(Ljava/lang/Object;)Ljava/lang/String;
    .registers 2
    .param p0, "object"    # Ljava/lang/Object;

    .line 126
    invoke-static {p0}, Lorg/apache/commons/lang/builder/ReflectionToStringBuilder;->toString(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public static reflectionToString(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;)Ljava/lang/String;
    .registers 3
    .param p0, "object"    # Ljava/lang/Object;
    .param p1, "style"    # Lorg/apache/commons/lang/builder/ToStringStyle;

    .line 138
    invoke-static {p0, p1}, Lorg/apache/commons/lang/builder/ReflectionToStringBuilder;->toString(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public static reflectionToString(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;Z)Ljava/lang/String;
    .registers 5
    .param p0, "object"    # Ljava/lang/Object;
    .param p1, "style"    # Lorg/apache/commons/lang/builder/ToStringStyle;
    .param p2, "outputTransients"    # Z

    .line 151
    const/4 v0, 0x0

    const/4 v1, 0x0

    invoke-static {p0, p1, p2, v0, v1}, Lorg/apache/commons/lang/builder/ReflectionToStringBuilder;->toString(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;ZZLjava/lang/Class;)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public static reflectionToString(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;ZLjava/lang/Class;)Ljava/lang/String;
    .registers 5
    .param p0, "object"    # Ljava/lang/Object;
    .param p1, "style"    # Lorg/apache/commons/lang/builder/ToStringStyle;
    .param p2, "outputTransients"    # Z
    .param p3, "reflectUpToClass"    # Ljava/lang/Class;

    .line 170
    const/4 v0, 0x0

    invoke-static {p0, p1, p2, v0, p3}, Lorg/apache/commons/lang/builder/ReflectionToStringBuilder;->toString(Ljava/lang/Object;Lorg/apache/commons/lang/builder/ToStringStyle;ZZLjava/lang/Class;)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public static setDefaultStyle(Lorg/apache/commons/lang/builder/ToStringStyle;)V
    .registers 3
    .param p0, "style"    # Lorg/apache/commons/lang/builder/ToStringStyle;

    .line 180
    if-eqz p0, :cond_5

    .line 183
    sput-object p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->defaultStyle:Lorg/apache/commons/lang/builder/ToStringStyle;

    .line 184
    return-void

    .line 181
    :cond_5
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "The style must not be null"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public append(B)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "value"    # B

    .line 296
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;B)V

    .line 297
    return-object p0
.end method

.method public append(C)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "value"    # C

    .line 324
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;C)V

    .line 325
    return-object p0
.end method

.method public append(D)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "value"    # D

    .line 352
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;D)V

    .line 353
    return-object p0
.end method

.method public append(F)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "value"    # F

    .line 380
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;F)V

    .line 381
    return-object p0
.end method

.method public append(I)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "value"    # I

    .line 408
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;I)V

    .line 409
    return-object p0
.end method

.method public append(J)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "value"    # J

    .line 436
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;J)V

    .line 437
    return-object p0
.end method

.method public append(Ljava/lang/Object;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "obj"    # Ljava/lang/Object;

    .line 464
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;Ljava/lang/Object;Ljava/lang/Boolean;)V

    .line 465
    return-object p0
.end method

.method public append(Ljava/lang/String;B)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # B

    .line 565
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;B)V

    .line 566
    return-object p0
.end method

.method public append(Ljava/lang/String;C)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # C

    .line 610
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;C)V

    .line 611
    return-object p0
.end method

.method public append(Ljava/lang/String;D)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # D

    .line 656
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2, p3}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;D)V

    .line 657
    return-object p0
.end method

.method public append(Ljava/lang/String;F)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # F

    .line 702
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;F)V

    .line 703
    return-object p0
.end method

.method public append(Ljava/lang/String;I)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # I

    .line 748
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;I)V

    .line 749
    return-object p0
.end method

.method public append(Ljava/lang/String;J)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # J

    .line 794
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2, p3}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;J)V

    .line 795
    return-object p0
.end method

.method public append(Ljava/lang/String;Ljava/lang/Object;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "obj"    # Ljava/lang/Object;

    .line 840
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;Ljava/lang/Object;Ljava/lang/Boolean;)V

    .line 841
    return-object p0
.end method

.method public append(Ljava/lang/String;Ljava/lang/Object;Z)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "obj"    # Ljava/lang/Object;
    .param p3, "fullDetail"    # Z

    .line 855
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;Ljava/lang/Object;Ljava/lang/Boolean;)V

    .line 856
    return-object p0
.end method

.method public append(Ljava/lang/String;S)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # S

    .line 901
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;S)V

    .line 902
    return-object p0
.end method

.method public append(Ljava/lang/String;Z)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "value"    # Z

    .line 519
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1, p2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;Z)V

    .line 520
    return-object p0
.end method

.method public append(Ljava/lang/String;[B)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [B

    .line 577
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[BLjava/lang/Boolean;)V

    .line 578
    return-object p0
.end method

.method public append(Ljava/lang/String;[BZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [B
    .param p3, "fullDetail"    # Z

    .line 597
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[BLjava/lang/Boolean;)V

    .line 598
    return-object p0
.end method

.method public append(Ljava/lang/String;[C)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [C

    .line 623
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[CLjava/lang/Boolean;)V

    .line 624
    return-object p0
.end method

.method public append(Ljava/lang/String;[CZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [C
    .param p3, "fullDetail"    # Z

    .line 643
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[CLjava/lang/Boolean;)V

    .line 644
    return-object p0
.end method

.method public append(Ljava/lang/String;[D)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [D

    .line 669
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[DLjava/lang/Boolean;)V

    .line 670
    return-object p0
.end method

.method public append(Ljava/lang/String;[DZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [D
    .param p3, "fullDetail"    # Z

    .line 689
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[DLjava/lang/Boolean;)V

    .line 690
    return-object p0
.end method

.method public append(Ljava/lang/String;[F)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [F

    .line 715
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[FLjava/lang/Boolean;)V

    .line 716
    return-object p0
.end method

.method public append(Ljava/lang/String;[FZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [F
    .param p3, "fullDetail"    # Z

    .line 735
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[FLjava/lang/Boolean;)V

    .line 736
    return-object p0
.end method

.method public append(Ljava/lang/String;[I)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [I

    .line 761
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[ILjava/lang/Boolean;)V

    .line 762
    return-object p0
.end method

.method public append(Ljava/lang/String;[IZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [I
    .param p3, "fullDetail"    # Z

    .line 781
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[ILjava/lang/Boolean;)V

    .line 782
    return-object p0
.end method

.method public append(Ljava/lang/String;[J)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [J

    .line 807
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[JLjava/lang/Boolean;)V

    .line 808
    return-object p0
.end method

.method public append(Ljava/lang/String;[JZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [J
    .param p3, "fullDetail"    # Z

    .line 827
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[JLjava/lang/Boolean;)V

    .line 828
    return-object p0
.end method

.method public append(Ljava/lang/String;[Ljava/lang/Object;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [Ljava/lang/Object;

    .line 868
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[Ljava/lang/Object;Ljava/lang/Boolean;)V

    .line 869
    return-object p0
.end method

.method public append(Ljava/lang/String;[Ljava/lang/Object;Z)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [Ljava/lang/Object;
    .param p3, "fullDetail"    # Z

    .line 888
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[Ljava/lang/Object;Ljava/lang/Boolean;)V

    .line 889
    return-object p0
.end method

.method public append(Ljava/lang/String;[S)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [S

    .line 914
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[SLjava/lang/Boolean;)V

    .line 915
    return-object p0
.end method

.method public append(Ljava/lang/String;[SZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [S
    .param p3, "fullDetail"    # Z

    .line 934
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[SLjava/lang/Boolean;)V

    .line 935
    return-object p0
.end method

.method public append(Ljava/lang/String;[Z)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 6
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [Z

    .line 532
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[ZLjava/lang/Boolean;)V

    .line 533
    return-object p0
.end method

.method public append(Ljava/lang/String;[ZZ)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 7
    .param p1, "fieldName"    # Ljava/lang/String;
    .param p2, "array"    # [Z
    .param p3, "fullDetail"    # Z

    .line 552
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-static {p3}, Lorg/apache/commons/lang/BooleanUtils;->toBooleanObject(Z)Ljava/lang/Boolean;

    move-result-object v2

    invoke-virtual {v0, v1, p1, p2, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[ZLjava/lang/Boolean;)V

    .line 553
    return-object p0
.end method

.method public append(S)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "value"    # S

    .line 492
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;S)V

    .line 493
    return-object p0
.end method

.method public append(Z)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "value"    # Z

    .line 268
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;Z)V

    .line 269
    return-object p0
.end method

.method public append([B)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [B

    .line 310
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[BLjava/lang/Boolean;)V

    .line 311
    return-object p0
.end method

.method public append([C)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [C

    .line 338
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[CLjava/lang/Boolean;)V

    .line 339
    return-object p0
.end method

.method public append([D)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [D

    .line 366
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[DLjava/lang/Boolean;)V

    .line 367
    return-object p0
.end method

.method public append([F)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [F

    .line 394
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[FLjava/lang/Boolean;)V

    .line 395
    return-object p0
.end method

.method public append([I)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [I

    .line 422
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[ILjava/lang/Boolean;)V

    .line 423
    return-object p0
.end method

.method public append([J)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [J

    .line 450
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[JLjava/lang/Boolean;)V

    .line 451
    return-object p0
.end method

.method public append([Ljava/lang/Object;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [Ljava/lang/Object;

    .line 478
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[Ljava/lang/Object;Ljava/lang/Boolean;)V

    .line 479
    return-object p0
.end method

.method public append([S)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [S

    .line 506
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[SLjava/lang/Boolean;)V

    .line 507
    return-object p0
.end method

.method public append([Z)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 5
    .param p1, "array"    # [Z

    .line 282
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    const/4 v2, 0x0

    invoke-virtual {v0, v1, v2, p1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->append(Ljava/lang/StringBuffer;Ljava/lang/String;[ZLjava/lang/Boolean;)V

    .line 283
    return-object p0
.end method

.method public appendAsObjectToString(Ljava/lang/Object;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 3
    .param p1, "object"    # Ljava/lang/Object;

    .line 948
    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getStringBuffer()Ljava/lang/StringBuffer;

    move-result-object v0

    invoke-static {v0, p1}, Lorg/apache/commons/lang/ObjectUtils;->appendIdentityToString(Ljava/lang/StringBuffer;Ljava/lang/Object;)Ljava/lang/StringBuffer;

    .line 949
    return-object p0
.end method

.method public appendSuper(Ljava/lang/String;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 4
    .param p1, "superToString"    # Ljava/lang/String;

    .line 967
    if-eqz p1, :cond_9

    .line 968
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->appendSuper(Ljava/lang/StringBuffer;Ljava/lang/String;)V

    .line 970
    :cond_9
    return-object p0
.end method

.method public appendToString(Ljava/lang/String;)Lorg/apache/commons/lang/builder/ToStringBuilder;
    .registers 4
    .param p1, "toString"    # Ljava/lang/String;

    .line 1001
    if-eqz p1, :cond_9

    .line 1002
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    iget-object v1, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    invoke-virtual {v0, v1, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->appendToString(Ljava/lang/StringBuffer;Ljava/lang/String;)V

    .line 1004
    :cond_9
    return-object p0
.end method

.method public getObject()Ljava/lang/Object;
    .registers 2

    .line 1014
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->object:Ljava/lang/Object;

    return-object v0
.end method

.method public getStringBuffer()Ljava/lang/StringBuffer;
    .registers 2

    .line 1023
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->buffer:Ljava/lang/StringBuffer;

    return-object v0
.end method

.method public getStyle()Lorg/apache/commons/lang/builder/ToStringStyle;
    .registers 2

    .line 1035
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    return-object v0
.end method

.method public toString()Ljava/lang/String;
    .registers 4

    .line 1049
    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getObject()Ljava/lang/Object;

    move-result-object v0

    if-nez v0, :cond_16

    .line 1050
    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getStringBuffer()Ljava/lang/StringBuffer;

    move-result-object v0

    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getStyle()Lorg/apache/commons/lang/builder/ToStringStyle;

    move-result-object v1

    invoke-virtual {v1}, Lorg/apache/commons/lang/builder/ToStringStyle;->getNullText()Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    goto :goto_23

    .line 1052
    :cond_16
    iget-object v0, p0, Lorg/apache/commons/lang/builder/ToStringBuilder;->style:Lorg/apache/commons/lang/builder/ToStringStyle;

    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getStringBuffer()Ljava/lang/StringBuffer;

    move-result-object v1

    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getObject()Ljava/lang/Object;

    move-result-object v2

    invoke-virtual {v0, v1, v2}, Lorg/apache/commons/lang/builder/ToStringStyle;->appendEnd(Ljava/lang/StringBuffer;Ljava/lang/Object;)V

    .line 1054
    :goto_23
    invoke-virtual {p0}, Lorg/apache/commons/lang/builder/ToStringBuilder;->getStringBuffer()Ljava/lang/StringBuffer;

    move-result-object v0

    invoke-virtual {v0}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method
