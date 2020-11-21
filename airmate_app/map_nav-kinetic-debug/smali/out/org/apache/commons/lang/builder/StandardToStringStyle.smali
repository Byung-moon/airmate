.class public Lorg/apache/commons/lang/builder/StandardToStringStyle;
.super Lorg/apache/commons/lang/builder/ToStringStyle;
.source "StandardToStringStyle.java"


# static fields
.field private static final serialVersionUID:J = 0x1L


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 47
    invoke-direct {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;-><init>()V

    .line 48
    return-void
.end method


# virtual methods
.method public getArrayEnd()Ljava/lang/String;
    .registers 2

    .line 226
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getArrayEnd()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getArraySeparator()Ljava/lang/String;
    .registers 2

    .line 249
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getArraySeparator()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getArrayStart()Ljava/lang/String;
    .registers 2

    .line 203
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getArrayStart()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getContentEnd()Ljava/lang/String;
    .registers 2

    .line 295
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getContentEnd()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getContentStart()Ljava/lang/String;
    .registers 2

    .line 272
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getContentStart()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getFieldNameValueSeparator()Ljava/lang/String;
    .registers 2

    .line 318
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getFieldNameValueSeparator()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getFieldSeparator()Ljava/lang/String;
    .registers 2

    .line 341
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getFieldSeparator()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getNullText()Ljava/lang/String;
    .registers 2

    .line 412
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getNullText()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getSizeEndText()Ljava/lang/String;
    .registers 2

    .line 467
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getSizeEndText()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getSizeStartText()Ljava/lang/String;
    .registers 2

    .line 438
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getSizeStartText()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getSummaryObjectEndText()Ljava/lang/String;
    .registers 2

    .line 525
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getSummaryObjectEndText()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getSummaryObjectStartText()Ljava/lang/String;
    .registers 2

    .line 496
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->getSummaryObjectStartText()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public isArrayContentDetail()Z
    .registers 2

    .line 183
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isArrayContentDetail()Z

    move-result v0

    return v0
.end method

.method public isDefaultFullDetail()Z
    .registers 2

    .line 162
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isDefaultFullDetail()Z

    move-result v0

    return v0
.end method

.method public isFieldSeparatorAtEnd()Z
    .registers 2

    .line 390
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isFieldSeparatorAtEnd()Z

    move-result v0

    return v0
.end method

.method public isFieldSeparatorAtStart()Z
    .registers 2

    .line 366
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isFieldSeparatorAtStart()Z

    move-result v0

    return v0
.end method

.method public isShortClassName()Z
    .registers 2

    .line 90
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isUseShortClassName()Z

    move-result v0

    return v0
.end method

.method public isUseClassName()Z
    .registers 2

    .line 58
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isUseClassName()Z

    move-result v0

    return v0
.end method

.method public isUseFieldNames()Z
    .registers 2

    .line 141
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isUseFieldNames()Z

    move-result v0

    return v0
.end method

.method public isUseIdentityHashCode()Z
    .registers 2

    .line 121
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isUseIdentityHashCode()Z

    move-result v0

    return v0
.end method

.method public isUseShortClassName()Z
    .registers 2

    .line 79
    invoke-super {p0}, Lorg/apache/commons/lang/builder/ToStringStyle;->isUseShortClassName()Z

    move-result v0

    return v0
.end method

.method public setArrayContentDetail(Z)V
    .registers 2
    .param p1, "arrayContentDetail"    # Z

    .line 192
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setArrayContentDetail(Z)V

    .line 193
    return-void
.end method

.method public setArrayEnd(Ljava/lang/String;)V
    .registers 2
    .param p1, "arrayEnd"    # Ljava/lang/String;

    .line 238
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setArrayEnd(Ljava/lang/String;)V

    .line 239
    return-void
.end method

.method public setArraySeparator(Ljava/lang/String;)V
    .registers 2
    .param p1, "arraySeparator"    # Ljava/lang/String;

    .line 261
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setArraySeparator(Ljava/lang/String;)V

    .line 262
    return-void
.end method

.method public setArrayStart(Ljava/lang/String;)V
    .registers 2
    .param p1, "arrayStart"    # Ljava/lang/String;

    .line 215
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setArrayStart(Ljava/lang/String;)V

    .line 216
    return-void
.end method

.method public setContentEnd(Ljava/lang/String;)V
    .registers 2
    .param p1, "contentEnd"    # Ljava/lang/String;

    .line 307
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setContentEnd(Ljava/lang/String;)V

    .line 308
    return-void
.end method

.method public setContentStart(Ljava/lang/String;)V
    .registers 2
    .param p1, "contentStart"    # Ljava/lang/String;

    .line 284
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setContentStart(Ljava/lang/String;)V

    .line 285
    return-void
.end method

.method public setDefaultFullDetail(Z)V
    .registers 2
    .param p1, "defaultFullDetail"    # Z

    .line 172
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setDefaultFullDetail(Z)V

    .line 173
    return-void
.end method

.method public setFieldNameValueSeparator(Ljava/lang/String;)V
    .registers 2
    .param p1, "fieldNameValueSeparator"    # Ljava/lang/String;

    .line 330
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setFieldNameValueSeparator(Ljava/lang/String;)V

    .line 331
    return-void
.end method

.method public setFieldSeparator(Ljava/lang/String;)V
    .registers 2
    .param p1, "fieldSeparator"    # Ljava/lang/String;

    .line 353
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setFieldSeparator(Ljava/lang/String;)V

    .line 354
    return-void
.end method

.method public setFieldSeparatorAtEnd(Z)V
    .registers 2
    .param p1, "fieldSeparatorAtEnd"    # Z

    .line 401
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setFieldSeparatorAtEnd(Z)V

    .line 402
    return-void
.end method

.method public setFieldSeparatorAtStart(Z)V
    .registers 2
    .param p1, "fieldSeparatorAtStart"    # Z

    .line 377
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setFieldSeparatorAtStart(Z)V

    .line 378
    return-void
.end method

.method public setNullText(Ljava/lang/String;)V
    .registers 2
    .param p1, "nullText"    # Ljava/lang/String;

    .line 424
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setNullText(Ljava/lang/String;)V

    .line 425
    return-void
.end method

.method public setShortClassName(Z)V
    .registers 2
    .param p1, "shortClassName"    # Z

    .line 111
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setUseShortClassName(Z)V

    .line 112
    return-void
.end method

.method public setSizeEndText(Ljava/lang/String;)V
    .registers 2
    .param p1, "sizeEndText"    # Ljava/lang/String;

    .line 482
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setSizeEndText(Ljava/lang/String;)V

    .line 483
    return-void
.end method

.method public setSizeStartText(Ljava/lang/String;)V
    .registers 2
    .param p1, "sizeStartText"    # Ljava/lang/String;

    .line 453
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setSizeStartText(Ljava/lang/String;)V

    .line 454
    return-void
.end method

.method public setSummaryObjectEndText(Ljava/lang/String;)V
    .registers 2
    .param p1, "summaryObjectEndText"    # Ljava/lang/String;

    .line 540
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setSummaryObjectEndText(Ljava/lang/String;)V

    .line 541
    return-void
.end method

.method public setSummaryObjectStartText(Ljava/lang/String;)V
    .registers 2
    .param p1, "summaryObjectStartText"    # Ljava/lang/String;

    .line 511
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setSummaryObjectStartText(Ljava/lang/String;)V

    .line 512
    return-void
.end method

.method public setUseClassName(Z)V
    .registers 2
    .param p1, "useClassName"    # Z

    .line 67
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setUseClassName(Z)V

    .line 68
    return-void
.end method

.method public setUseFieldNames(Z)V
    .registers 2
    .param p1, "useFieldNames"    # Z

    .line 150
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setUseFieldNames(Z)V

    .line 151
    return-void
.end method

.method public setUseIdentityHashCode(Z)V
    .registers 2
    .param p1, "useIdentityHashCode"    # Z

    .line 130
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setUseIdentityHashCode(Z)V

    .line 131
    return-void
.end method

.method public setUseShortClassName(Z)V
    .registers 2
    .param p1, "useShortClassName"    # Z

    .line 100
    invoke-super {p0, p1}, Lorg/apache/commons/lang/builder/ToStringStyle;->setUseShortClassName(Z)V

    .line 101
    return-void
.end method
