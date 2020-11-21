.class public Landroid/support/graphics/drawable/AnimatorInflaterCompat;
.super Ljava/lang/Object;
.source "AnimatorInflaterCompat.java"


# annotations
.annotation build Landroid/support/annotation/RestrictTo;
    value = {
        .enum Landroid/support/annotation/RestrictTo$Scope;->LIBRARY_GROUP:Landroid/support/annotation/RestrictTo$Scope;
    }
.end annotation

.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Landroid/support/graphics/drawable/AnimatorInflaterCompat$PathDataEvaluator;
    }
.end annotation


# static fields
.field private static final DBG_ANIMATOR_INFLATER:Z = false

.field private static final MAX_NUM_POINTS:I = 0x64

.field private static final TAG:Ljava/lang/String; = "AnimatorInflater"

.field private static final TOGETHER:I = 0x0

.field private static final VALUE_TYPE_COLOR:I = 0x3

.field private static final VALUE_TYPE_FLOAT:I = 0x0

.field private static final VALUE_TYPE_INT:I = 0x1

.field private static final VALUE_TYPE_PATH:I = 0x2

.field private static final VALUE_TYPE_UNDEFINED:I = 0x4


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 68
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 156
    return-void
.end method

.method private static createAnimatorFromXml(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;F)Landroid/animation/Animator;
    .registers 13
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .param p4, "pixelSize"    # F
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/xmlpull/v1/XmlPullParserException;,
            Ljava/io/IOException;
        }
    .end annotation

    .line 500
    invoke-static {p3}, Landroid/util/Xml;->asAttributeSet(Lorg/xmlpull/v1/XmlPullParser;)Landroid/util/AttributeSet;

    move-result-object v4

    const/4 v5, 0x0

    const/4 v6, 0x0

    move-object v0, p0

    move-object v1, p1

    move-object v2, p2

    move-object v3, p3

    move v7, p4

    invoke-static/range {v0 .. v7}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->createAnimatorFromXml(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Landroid/util/AttributeSet;Landroid/animation/AnimatorSet;IF)Landroid/animation/Animator;

    move-result-object v0

    return-object v0
.end method

.method private static createAnimatorFromXml(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Landroid/util/AttributeSet;Landroid/animation/AnimatorSet;IF)Landroid/animation/Animator;
    .registers 28
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .param p4, "attrs"    # Landroid/util/AttributeSet;
    .param p5, "parent"    # Landroid/animation/AnimatorSet;
    .param p6, "sequenceOrdering"    # I
    .param p7, "pixelSize"    # F
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/xmlpull/v1/XmlPullParserException;,
            Ljava/io/IOException;
        }
    .end annotation

    .line 508
    move-object/from16 v8, p1

    move-object/from16 v9, p2

    move-object/from16 v10, p3

    move-object/from16 v11, p5

    const/4 v0, 0x0

    .line 509
    .local v0, "anim":Landroid/animation/Animator;
    const/4 v1, 0x0

    .line 513
    .local v1, "childAnims":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Animator;>;"
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getDepth()I

    move-result v2

    move-object v7, v0

    move-object v12, v1

    .end local v0    # "anim":Landroid/animation/Animator;
    .end local v1    # "childAnims":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Animator;>;"
    .local v2, "depth":I
    .local v7, "anim":Landroid/animation/Animator;
    .local v12, "childAnims":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Animator;>;"
    :goto_10
    move v13, v2

    .line 515
    .end local v2    # "depth":I
    .local v13, "depth":I
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->next()I

    move-result v0

    move v14, v0

    .local v14, "type":I
    const/4 v1, 0x3

    if-ne v0, v1, :cond_24

    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getDepth()I

    move-result v0

    if-le v0, v13, :cond_20

    goto :goto_24

    .line 558
    :cond_20
    move-object/from16 v1, p0

    goto/16 :goto_fa

    .line 515
    :cond_24
    :goto_24
    const/4 v0, 0x1

    if-eq v14, v0, :cond_20

    .line 518
    const/4 v0, 0x2

    if-eq v14, v0, :cond_2d

    .line 519
    nop

    .line 513
    .end local v13    # "depth":I
    .end local v14    # "type":I
    .restart local v2    # "depth":I
    :goto_2b
    move v2, v13

    goto :goto_10

    .line 522
    .end local v2    # "depth":I
    .restart local v13    # "depth":I
    .restart local v14    # "type":I
    :cond_2d
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getName()Ljava/lang/String;

    move-result-object v15

    .line 523
    .local v15, "name":Ljava/lang/String;
    const/16 v16, 0x0

    .line 525
    .local v16, "gotValues":Z
    const-string v0, "objectAnimator"

    invoke-virtual {v15, v0}, Ljava/lang/String;->equals(Ljava/lang/Object;)Z

    move-result v0

    if-eqz v0, :cond_50

    .line 526
    move-object/from16 v0, p0

    move-object/from16 v1, p1

    move-object/from16 v2, p2

    move-object/from16 v3, p4

    move/from16 v4, p7

    move-object/from16 v5, p3

    invoke-static/range {v0 .. v5}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadObjectAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;FLorg/xmlpull/v1/XmlPullParser;)Landroid/animation/ObjectAnimator;

    move-result-object v0

    .line 551
    :goto_4b
    move-object/from16 v1, p0

    move-object v7, v0

    goto/16 :goto_cc

    .line 527
    :cond_50
    const-string v0, "animator"

    invoke-virtual {v15, v0}, Ljava/lang/String;->equals(Ljava/lang/Object;)Z

    move-result v0

    if-eqz v0, :cond_6a

    .line 528
    const/4 v4, 0x0

    move-object/from16 v0, p0

    move-object/from16 v1, p1

    move-object/from16 v2, p2

    move-object/from16 v3, p4

    move/from16 v5, p7

    move-object/from16 v6, p3

    invoke-static/range {v0 .. v6}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;Landroid/animation/ValueAnimator;FLorg/xmlpull/v1/XmlPullParser;)Landroid/animation/ValueAnimator;

    move-result-object v0

    goto :goto_4b

    .line 529
    :cond_6a
    const-string v0, "set"

    invoke-virtual {v15, v0}, Ljava/lang/String;->equals(Ljava/lang/Object;)Z

    move-result v0

    if-eqz v0, :cond_a8

    .line 530
    new-instance v0, Landroid/animation/AnimatorSet;

    invoke-direct {v0}, Landroid/animation/AnimatorSet;-><init>()V

    move-object/from16 v17, v0

    .line 531
    .end local v7    # "anim":Landroid/animation/Animator;
    .local v17, "anim":Landroid/animation/Animator;
    sget-object v0, Landroid/support/graphics/drawable/AndroidResources;->STYLEABLE_ANIMATOR_SET:[I

    move-object/from16 v7, p4

    invoke-static {v8, v9, v7, v0}, Landroid/support/v4/content/res/TypedArrayUtils;->obtainAttributes(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;[I)Landroid/content/res/TypedArray;

    move-result-object v6

    .line 534
    .local v6, "a":Landroid/content/res/TypedArray;
    const-string v0, "ordering"

    const/4 v1, 0x0

    invoke-static {v6, v10, v0, v1, v1}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v18

    .line 537
    .local v18, "ordering":I
    move-object/from16 v5, v17

    check-cast v5, Landroid/animation/AnimatorSet;

    move-object/from16 v0, p0

    move-object/from16 v1, p1

    move-object/from16 v2, p2

    move-object/from16 v3, p3

    move-object/from16 v4, p4

    move-object/from16 v19, v6

    .end local v6    # "a":Landroid/content/res/TypedArray;
    .local v19, "a":Landroid/content/res/TypedArray;
    move/from16 v6, v18

    move/from16 v7, p7

    invoke-static/range {v0 .. v7}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->createAnimatorFromXml(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Landroid/util/AttributeSet;Landroid/animation/AnimatorSet;IF)Landroid/animation/Animator;

    .line 539
    invoke-virtual/range {v19 .. v19}, Landroid/content/res/TypedArray;->recycle()V

    .line 540
    .end local v18    # "ordering":I
    .end local v19    # "a":Landroid/content/res/TypedArray;
    nop

    .line 551
    move-object/from16 v1, p0

    move-object/from16 v7, v17

    goto :goto_cc

    .line 540
    .end local v17    # "anim":Landroid/animation/Animator;
    .restart local v7    # "anim":Landroid/animation/Animator;
    :cond_a8
    const-string v0, "propertyValuesHolder"

    invoke-virtual {v15, v0}, Ljava/lang/String;->equals(Ljava/lang/Object;)Z

    move-result v0

    if-eqz v0, :cond_dd

    .line 541
    nop

    .line 542
    invoke-static/range {p3 .. p3}, Landroid/util/Xml;->asAttributeSet(Lorg/xmlpull/v1/XmlPullParser;)Landroid/util/AttributeSet;

    move-result-object v0

    .line 541
    move-object/from16 v1, p0

    invoke-static {v1, v8, v9, v10, v0}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadValues(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Landroid/util/AttributeSet;)[Landroid/animation/PropertyValuesHolder;

    move-result-object v0

    .line 543
    .local v0, "values":[Landroid/animation/PropertyValuesHolder;
    if-eqz v0, :cond_c9

    if-eqz v7, :cond_c9

    instance-of v2, v7, Landroid/animation/ValueAnimator;

    if-eqz v2, :cond_c9

    .line 544
    move-object v2, v7

    check-cast v2, Landroid/animation/ValueAnimator;

    invoke-virtual {v2, v0}, Landroid/animation/ValueAnimator;->setValues([Landroid/animation/PropertyValuesHolder;)V

    .line 546
    :cond_c9
    const/16 v16, 0x1

    .line 547
    .end local v0    # "values":[Landroid/animation/PropertyValuesHolder;
    nop

    .line 551
    :goto_cc
    if-eqz v11, :cond_db

    if-nez v16, :cond_db

    .line 552
    if-nez v12, :cond_d8

    .line 553
    new-instance v0, Ljava/util/ArrayList;

    invoke-direct {v0}, Ljava/util/ArrayList;-><init>()V

    move-object v12, v0

    .line 555
    :cond_d8
    invoke-virtual {v12, v7}, Ljava/util/ArrayList;->add(Ljava/lang/Object;)Z

    .line 557
    .end local v15    # "name":Ljava/lang/String;
    .end local v16    # "gotValues":Z
    :cond_db
    goto/16 :goto_2b

    .line 548
    .restart local v15    # "name":Ljava/lang/String;
    .restart local v16    # "gotValues":Z
    :cond_dd
    move-object/from16 v1, p0

    new-instance v0, Ljava/lang/RuntimeException;

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    const-string v3, "Unknown animator name: "

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getName()Ljava/lang/String;

    move-result-object v3

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v0, v2}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 558
    .end local v15    # "name":Ljava/lang/String;
    .end local v16    # "gotValues":Z
    :goto_fa
    if-eqz v11, :cond_125

    if-eqz v12, :cond_125

    .line 559
    invoke-virtual {v12}, Ljava/util/ArrayList;->size()I

    move-result v0

    new-array v0, v0, [Landroid/animation/Animator;

    .line 560
    .local v0, "animsArray":[Landroid/animation/Animator;
    const/4 v2, 0x0

    .line 561
    .local v2, "index":I
    invoke-virtual {v12}, Ljava/util/ArrayList;->iterator()Ljava/util/Iterator;

    move-result-object v3

    :goto_109
    invoke-interface {v3}, Ljava/util/Iterator;->hasNext()Z

    move-result v4

    if-eqz v4, :cond_11c

    invoke-interface {v3}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v4

    check-cast v4, Landroid/animation/Animator;

    .line 562
    .local v4, "a":Landroid/animation/Animator;
    add-int/lit8 v5, v2, 0x1

    .local v5, "index":I
    aput-object v4, v0, v2

    .line 563
    .end local v2    # "index":I
    .end local v4    # "a":Landroid/animation/Animator;
    nop

    .line 561
    move v2, v5

    goto :goto_109

    .line 564
    .end local v5    # "index":I
    .restart local v2    # "index":I
    :cond_11c
    if-nez p6, :cond_122

    .line 565
    invoke-virtual {v11, v0}, Landroid/animation/AnimatorSet;->playTogether([Landroid/animation/Animator;)V

    goto :goto_125

    .line 567
    :cond_122
    invoke-virtual {v11, v0}, Landroid/animation/AnimatorSet;->playSequentially([Landroid/animation/Animator;)V

    .line 570
    .end local v0    # "animsArray":[Landroid/animation/Animator;
    .end local v2    # "index":I
    :cond_125
    :goto_125
    return-object v7
.end method

.method private static createNewKeyframe(Landroid/animation/Keyframe;F)Landroid/animation/Keyframe;
    .registers 4
    .param p0, "sampleKeyframe"    # Landroid/animation/Keyframe;
    .param p1, "fraction"    # F

    .line 778
    invoke-virtual {p0}, Landroid/animation/Keyframe;->getType()Ljava/lang/Class;

    move-result-object v0

    sget-object v1, Ljava/lang/Float;->TYPE:Ljava/lang/Class;

    if-ne v0, v1, :cond_d

    .line 779
    invoke-static {p1}, Landroid/animation/Keyframe;->ofFloat(F)Landroid/animation/Keyframe;

    move-result-object v0

    goto :goto_1e

    .line 780
    :cond_d
    invoke-virtual {p0}, Landroid/animation/Keyframe;->getType()Ljava/lang/Class;

    move-result-object v0

    sget-object v1, Ljava/lang/Integer;->TYPE:Ljava/lang/Class;

    if-ne v0, v1, :cond_1a

    .line 781
    invoke-static {p1}, Landroid/animation/Keyframe;->ofInt(F)Landroid/animation/Keyframe;

    move-result-object v0

    goto :goto_1e

    .line 782
    :cond_1a
    invoke-static {p1}, Landroid/animation/Keyframe;->ofObject(F)Landroid/animation/Keyframe;

    move-result-object v0

    :goto_1e
    return-object v0
.end method

.method private static distributeKeyframes([Landroid/animation/Keyframe;FII)V
    .registers 9
    .param p0, "keyframes"    # [Landroid/animation/Keyframe;
    .param p1, "gap"    # F
    .param p2, "startIndex"    # I
    .param p3, "endIndex"    # I

    .line 805
    sub-int v0, p3, p2

    add-int/lit8 v0, v0, 0x2

    .line 806
    .local v0, "count":I
    int-to-float v1, v0

    div-float v1, p1, v1

    .line 807
    .local v1, "increment":F
    move v2, p2

    .local v2, "i":I
    :goto_8
    if-gt v2, p3, :cond_1b

    .line 808
    aget-object v3, p0, v2

    add-int/lit8 v4, v2, -0x1

    aget-object v4, p0, v4

    invoke-virtual {v4}, Landroid/animation/Keyframe;->getFraction()F

    move-result v4

    add-float/2addr v4, v1

    invoke-virtual {v3, v4}, Landroid/animation/Keyframe;->setFraction(F)V

    .line 807
    add-int/lit8 v2, v2, 0x1

    goto :goto_8

    .line 810
    .end local v2    # "i":I
    :cond_1b
    return-void
.end method

.method private static dumpKeyframes([Ljava/lang/Object;Ljava/lang/String;)V
    .registers 9
    .param p0, "keyframes"    # [Ljava/lang/Object;
    .param p1, "header"    # Ljava/lang/String;

    .line 672
    if-eqz p0, :cond_63

    array-length v0, p0

    if-nez v0, :cond_6

    goto :goto_63

    .line 675
    :cond_6
    const-string v0, "AnimatorInflater"

    invoke-static {v0, p1}, Landroid/util/Log;->d(Ljava/lang/String;Ljava/lang/String;)I

    .line 676
    array-length v0, p0

    .line 677
    .local v0, "count":I
    const/4 v1, 0x0

    .local v1, "i":I
    :goto_d
    if-ge v1, v0, :cond_62

    .line 678
    aget-object v2, p0, v1

    check-cast v2, Landroid/animation/Keyframe;

    .line 679
    .local v2, "keyframe":Landroid/animation/Keyframe;
    const-string v3, "AnimatorInflater"

    new-instance v4, Ljava/lang/StringBuilder;

    invoke-direct {v4}, Ljava/lang/StringBuilder;-><init>()V

    const-string v5, "Keyframe "

    invoke-virtual {v4, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v4, v1}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    const-string v5, ": fraction "

    invoke-virtual {v4, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 680
    invoke-virtual {v2}, Landroid/animation/Keyframe;->getFraction()F

    move-result v5

    const/4 v6, 0x0

    cmpg-float v5, v5, v6

    if-gez v5, :cond_33

    const-string v5, "null"

    goto :goto_3b

    :cond_33
    invoke-virtual {v2}, Landroid/animation/Keyframe;->getFraction()F

    move-result v5

    invoke-static {v5}, Ljava/lang/Float;->valueOf(F)Ljava/lang/Float;

    move-result-object v5

    :goto_3b
    invoke-virtual {v4, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/Object;)Ljava/lang/StringBuilder;

    const-string v5, ", "

    invoke-virtual {v4, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v5, ", value : "

    invoke-virtual {v4, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 681
    invoke-virtual {v2}, Landroid/animation/Keyframe;->hasValue()Z

    move-result v5

    if-eqz v5, :cond_53

    invoke-virtual {v2}, Landroid/animation/Keyframe;->getValue()Ljava/lang/Object;

    move-result-object v5

    goto :goto_55

    :cond_53
    const-string v5, "null"

    :goto_55
    invoke-virtual {v4, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/Object;)Ljava/lang/StringBuilder;

    invoke-virtual {v4}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v4

    .line 679
    invoke-static {v3, v4}, Landroid/util/Log;->d(Ljava/lang/String;Ljava/lang/String;)I

    .line 677
    .end local v2    # "keyframe":Landroid/animation/Keyframe;
    add-int/lit8 v1, v1, 0x1

    goto :goto_d

    .line 683
    .end local v1    # "i":I
    :cond_62
    return-void

    .line 673
    .end local v0    # "count":I
    :cond_63
    :goto_63
    return-void
.end method

.method private static getPVH(Landroid/content/res/TypedArray;IIILjava/lang/String;)Landroid/animation/PropertyValuesHolder;
    .registers 27
    .param p0, "styledAttributes"    # Landroid/content/res/TypedArray;
    .param p1, "valueType"    # I
    .param p2, "valueFromId"    # I
    .param p3, "valueToId"    # I
    .param p4, "propertyName"    # Ljava/lang/String;

    .line 206
    move-object/from16 v0, p0

    move/from16 v1, p2

    move/from16 v2, p3

    move-object/from16 v3, p4

    invoke-virtual {v0, v1}, Landroid/content/res/TypedArray;->peekValue(I)Landroid/util/TypedValue;

    move-result-object v4

    .line 207
    .local v4, "tvFrom":Landroid/util/TypedValue;
    if-eqz v4, :cond_10

    const/4 v7, 0x1

    goto :goto_11

    :cond_10
    const/4 v7, 0x0

    .line 208
    .local v7, "hasFrom":Z
    :goto_11
    if-eqz v7, :cond_16

    iget v8, v4, Landroid/util/TypedValue;->type:I

    goto :goto_17

    :cond_16
    const/4 v8, 0x0

    .line 209
    .local v8, "fromType":I
    :goto_17
    invoke-virtual {v0, v2}, Landroid/content/res/TypedArray;->peekValue(I)Landroid/util/TypedValue;

    move-result-object v9

    .line 210
    .local v9, "tvTo":Landroid/util/TypedValue;
    if-eqz v9, :cond_1f

    const/4 v10, 0x1

    goto :goto_20

    :cond_1f
    const/4 v10, 0x0

    .line 211
    .local v10, "hasTo":Z
    :goto_20
    if-eqz v10, :cond_25

    iget v11, v9, Landroid/util/TypedValue;->type:I

    goto :goto_26

    :cond_25
    const/4 v11, 0x0

    .line 213
    .local v11, "toType":I
    :goto_26
    const/4 v12, 0x4

    move/from16 v13, p1

    if-ne v13, v12, :cond_3f

    .line 215
    if-eqz v7, :cond_33

    invoke-static {v8}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v12

    if-nez v12, :cond_3b

    :cond_33
    if-eqz v10, :cond_3d

    invoke-static {v11}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v12

    if-eqz v12, :cond_3d

    .line 216
    :cond_3b
    const/4 v12, 0x3

    goto :goto_40

    .line 218
    :cond_3d
    const/4 v12, 0x0

    goto :goto_40

    .line 222
    :cond_3f
    move v12, v13

    .end local p1    # "valueType":I
    .local v12, "valueType":I
    :goto_40
    if-nez v12, :cond_44

    const/4 v13, 0x1

    goto :goto_45

    :cond_44
    const/4 v13, 0x0

    .line 224
    .local v13, "getFloats":Z
    :goto_45
    const/4 v14, 0x0

    .line 226
    .local v14, "returnValue":Landroid/animation/PropertyValuesHolder;
    const/4 v15, 0x2

    if-ne v12, v15, :cond_de

    .line 227
    invoke-virtual {v0, v1}, Landroid/content/res/TypedArray;->getString(I)Ljava/lang/String;

    move-result-object v5

    .line 228
    .local v5, "fromString":Ljava/lang/String;
    invoke-virtual {v0, v2}, Landroid/content/res/TypedArray;->getString(I)Ljava/lang/String;

    move-result-object v6

    .line 230
    .local v6, "toString":Ljava/lang/String;
    nop

    .line 231
    invoke-static {v5}, Landroid/support/v4/graphics/PathParser;->createNodesFromPathData(Ljava/lang/String;)[Landroid/support/v4/graphics/PathParser$PathDataNode;

    move-result-object v15

    .line 232
    .local v15, "nodesFrom":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    nop

    .line 233
    move-object/from16 v18, v4

    .end local v4    # "tvFrom":Landroid/util/TypedValue;
    .local v18, "tvFrom":Landroid/util/TypedValue;
    invoke-static {v6}, Landroid/support/v4/graphics/PathParser;->createNodesFromPathData(Ljava/lang/String;)[Landroid/support/v4/graphics/PathParser$PathDataNode;

    move-result-object v4

    .line 234
    .local v4, "nodesTo":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    if-nez v15, :cond_6a

    if-eqz v4, :cond_62

    goto :goto_6a

    .line 254
    .end local v4    # "nodesTo":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    .end local v5    # "fromString":Ljava/lang/String;
    .end local v6    # "toString":Ljava/lang/String;
    .end local v15    # "nodesFrom":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    :cond_62
    move-object/from16 v19, v9

    move/from16 v21, v11

    move-object/from16 v20, v14

    goto/16 :goto_d6

    .line 235
    .restart local v4    # "nodesTo":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    .restart local v5    # "fromString":Ljava/lang/String;
    .restart local v6    # "toString":Ljava/lang/String;
    .restart local v15    # "nodesFrom":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    :cond_6a
    :goto_6a
    move-object/from16 v19, v9

    .end local v9    # "tvTo":Landroid/util/TypedValue;
    .local v19, "tvTo":Landroid/util/TypedValue;
    const/4 v9, 0x0

    if-eqz v15, :cond_c0

    .line 236
    move-object/from16 v20, v14

    .end local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .local v20, "returnValue":Landroid/animation/PropertyValuesHolder;
    new-instance v14, Landroid/support/graphics/drawable/AnimatorInflaterCompat$PathDataEvaluator;

    invoke-direct {v14, v9}, Landroid/support/graphics/drawable/AnimatorInflaterCompat$PathDataEvaluator;-><init>(Landroid/support/graphics/drawable/AnimatorInflaterCompat$1;)V

    move-object v9, v14

    .line 237
    .local v9, "evaluator":Landroid/animation/TypeEvaluator;
    if-eqz v4, :cond_b2

    .line 238
    invoke-static {v15, v4}, Landroid/support/v4/graphics/PathParser;->canMorph([Landroid/support/v4/graphics/PathParser$PathDataNode;[Landroid/support/v4/graphics/PathParser$PathDataNode;)Z

    move-result v14

    if-eqz v14, :cond_91

    .line 242
    const/4 v14, 0x2

    new-array v14, v14, [Ljava/lang/Object;

    const/16 v17, 0x0

    aput-object v15, v14, v17

    const/16 v16, 0x1

    aput-object v4, v14, v16

    invoke-static {v3, v9, v14}, Landroid/animation/PropertyValuesHolder;->ofObject(Ljava/lang/String;Landroid/animation/TypeEvaluator;[Ljava/lang/Object;)Landroid/animation/PropertyValuesHolder;

    move-result-object v14

    .line 248
    move/from16 v21, v11

    goto :goto_bf

    .line 239
    :cond_91
    new-instance v14, Landroid/view/InflateException;

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    move/from16 v21, v11

    .end local v11    # "toType":I
    .local v21, "toType":I
    const-string v11, " Can\'t morph from "

    invoke-virtual {v2, v11}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, v5}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v11, " to "

    invoke-virtual {v2, v11}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, v6}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v14, v2}, Landroid/view/InflateException;-><init>(Ljava/lang/String;)V

    throw v14

    .line 245
    .end local v21    # "toType":I
    .restart local v11    # "toType":I
    :cond_b2
    move/from16 v21, v11

    .end local v11    # "toType":I
    .restart local v21    # "toType":I
    const/4 v2, 0x1

    new-array v2, v2, [Ljava/lang/Object;

    const/4 v11, 0x0

    aput-object v15, v2, v11

    invoke-static {v3, v9, v2}, Landroid/animation/PropertyValuesHolder;->ofObject(Ljava/lang/String;Landroid/animation/TypeEvaluator;[Ljava/lang/Object;)Landroid/animation/PropertyValuesHolder;

    move-result-object v2

    .line 248
    move-object v14, v2

    .end local v9    # "evaluator":Landroid/animation/TypeEvaluator;
    .end local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    :goto_bf
    goto :goto_d8

    .end local v21    # "toType":I
    .restart local v11    # "toType":I
    :cond_c0
    move/from16 v21, v11

    move-object/from16 v20, v14

    .end local v11    # "toType":I
    .end local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v21    # "toType":I
    if-eqz v4, :cond_d6

    .line 249
    new-instance v2, Landroid/support/graphics/drawable/AnimatorInflaterCompat$PathDataEvaluator;

    invoke-direct {v2, v9}, Landroid/support/graphics/drawable/AnimatorInflaterCompat$PathDataEvaluator;-><init>(Landroid/support/graphics/drawable/AnimatorInflaterCompat$1;)V

    .line 250
    .local v2, "evaluator":Landroid/animation/TypeEvaluator;
    const/4 v9, 0x1

    new-array v9, v9, [Ljava/lang/Object;

    const/4 v11, 0x0

    aput-object v4, v9, v11

    invoke-static {v3, v2, v9}, Landroid/animation/PropertyValuesHolder;->ofObject(Ljava/lang/String;Landroid/animation/TypeEvaluator;[Ljava/lang/Object;)Landroid/animation/PropertyValuesHolder;

    move-result-object v14

    goto :goto_d8

    .line 254
    .end local v2    # "evaluator":Landroid/animation/TypeEvaluator;
    .end local v4    # "nodesTo":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    .end local v5    # "fromString":Ljava/lang/String;
    .end local v6    # "toString":Ljava/lang/String;
    .end local v15    # "nodesFrom":[Landroid/support/v4/graphics/PathParser$PathDataNode;
    .end local v19    # "tvTo":Landroid/util/TypedValue;
    .end local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .end local v21    # "toType":I
    .local v9, "tvTo":Landroid/util/TypedValue;
    .restart local v11    # "toType":I
    .restart local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    :cond_d6
    :goto_d6
    move-object/from16 v14, v20

    .line 330
    .end local v9    # "tvTo":Landroid/util/TypedValue;
    .end local v11    # "toType":I
    .restart local v19    # "tvTo":Landroid/util/TypedValue;
    .restart local v21    # "toType":I
    :goto_d8
    move/from16 v9, v21

    move/from16 v11, p3

    goto/16 :goto_1d2

    .line 255
    .end local v18    # "tvFrom":Landroid/util/TypedValue;
    .end local v19    # "tvTo":Landroid/util/TypedValue;
    .end local v21    # "toType":I
    .local v4, "tvFrom":Landroid/util/TypedValue;
    .restart local v9    # "tvTo":Landroid/util/TypedValue;
    .restart local v11    # "toType":I
    :cond_de
    move-object/from16 v18, v4

    move-object/from16 v19, v9

    move/from16 v21, v11

    move-object/from16 v20, v14

    .end local v4    # "tvFrom":Landroid/util/TypedValue;
    .end local v9    # "tvTo":Landroid/util/TypedValue;
    .end local v11    # "toType":I
    .end local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v18    # "tvFrom":Landroid/util/TypedValue;
    .restart local v19    # "tvTo":Landroid/util/TypedValue;
    .restart local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v21    # "toType":I
    const/4 v2, 0x0

    .line 257
    .restart local v2    # "evaluator":Landroid/animation/TypeEvaluator;
    const/4 v4, 0x3

    if-ne v12, v4, :cond_ee

    .line 259
    invoke-static {}, Landroid/support/graphics/drawable/ArgbEvaluator;->getInstance()Landroid/support/graphics/drawable/ArgbEvaluator;

    move-result-object v2

    .line 261
    :cond_ee
    const/4 v4, 0x5

    const/4 v5, 0x0

    if-eqz v13, :cond_14c

    .line 264
    if-eqz v7, :cond_130

    .line 265
    if-ne v8, v4, :cond_fb

    .line 266
    invoke-virtual {v0, v1, v5}, Landroid/content/res/TypedArray;->getDimension(IF)F

    move-result v6

    goto :goto_ff

    .line 268
    :cond_fb
    invoke-virtual {v0, v1, v5}, Landroid/content/res/TypedArray;->getFloat(IF)F

    move-result v6

    .line 270
    .local v6, "valueFrom":F
    :goto_ff
    if-eqz v10, :cond_121

    .line 271
    move/from16 v9, v21

    .end local v21    # "toType":I
    .local v9, "toType":I
    if-ne v9, v4, :cond_10c

    .line 272
    move/from16 v11, p3

    invoke-virtual {v0, v11, v5}, Landroid/content/res/TypedArray;->getDimension(IF)F

    move-result v4

    goto :goto_112

    .line 274
    :cond_10c
    move/from16 v11, p3

    invoke-virtual {v0, v11, v5}, Landroid/content/res/TypedArray;->getFloat(IF)F

    move-result v4

    .line 276
    .local v4, "valueTo":F
    :goto_112
    const/4 v5, 0x2

    new-array v5, v5, [F

    const/4 v14, 0x0

    aput v6, v5, v14

    const/4 v15, 0x1

    aput v4, v5, v15

    invoke-static {v3, v5}, Landroid/animation/PropertyValuesHolder;->ofFloat(Ljava/lang/String;[F)Landroid/animation/PropertyValuesHolder;

    move-result-object v5

    .line 289
    move-object v14, v5

    goto :goto_14a

    .line 279
    .end local v4    # "valueTo":F
    .end local v9    # "toType":I
    .restart local v21    # "toType":I
    :cond_121
    move/from16 v9, v21

    move/from16 v11, p3

    const/4 v14, 0x0

    const/4 v15, 0x1

    .end local v21    # "toType":I
    .restart local v9    # "toType":I
    new-array v4, v15, [F

    aput v6, v4, v14

    invoke-static {v3, v4}, Landroid/animation/PropertyValuesHolder;->ofFloat(Ljava/lang/String;[F)Landroid/animation/PropertyValuesHolder;

    move-result-object v4

    goto :goto_149

    .line 282
    .end local v6    # "valueFrom":F
    .end local v9    # "toType":I
    .restart local v21    # "toType":I
    :cond_130
    move/from16 v9, v21

    move/from16 v11, p3

    .end local v21    # "toType":I
    .restart local v9    # "toType":I
    if-ne v9, v4, :cond_13b

    .line 283
    invoke-virtual {v0, v11, v5}, Landroid/content/res/TypedArray;->getDimension(IF)F

    move-result v4

    goto :goto_13f

    .line 285
    :cond_13b
    invoke-virtual {v0, v11, v5}, Landroid/content/res/TypedArray;->getFloat(IF)F

    move-result v4

    .line 287
    .restart local v4    # "valueTo":F
    :goto_13f
    const/4 v5, 0x1

    new-array v5, v5, [F

    const/4 v6, 0x0

    aput v4, v5, v6

    invoke-static {v3, v5}, Landroid/animation/PropertyValuesHolder;->ofFloat(Ljava/lang/String;[F)Landroid/animation/PropertyValuesHolder;

    move-result-object v4

    .line 289
    .end local v4    # "valueTo":F
    .end local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    :goto_149
    move-object v14, v4

    :goto_14a
    goto/16 :goto_1cb

    .line 292
    .end local v9    # "toType":I
    .end local v14    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .restart local v21    # "toType":I
    :cond_14c
    move/from16 v9, v21

    move/from16 v11, p3

    .end local v21    # "toType":I
    .restart local v9    # "toType":I
    if-eqz v7, :cond_1a2

    .line 293
    if-ne v8, v4, :cond_15b

    .line 294
    invoke-virtual {v0, v1, v5}, Landroid/content/res/TypedArray;->getDimension(IF)F

    move-result v6

    float-to-int v6, v6

    .line 298
    move v14, v6

    goto :goto_16c

    .line 295
    :cond_15b
    invoke-static {v8}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v6

    if-eqz v6, :cond_167

    .line 296
    const/4 v6, 0x0

    invoke-virtual {v0, v1, v6}, Landroid/content/res/TypedArray;->getColor(II)I

    move-result v14

    goto :goto_16c

    .line 298
    :cond_167
    const/4 v6, 0x0

    invoke-virtual {v0, v1, v6}, Landroid/content/res/TypedArray;->getInt(II)I

    move-result v14

    .local v14, "valueFrom":I
    :goto_16c
    move v6, v14

    .line 300
    .end local v14    # "valueFrom":I
    .local v6, "valueFrom":I
    if-eqz v10, :cond_197

    .line 301
    if-ne v9, v4, :cond_179

    .line 302
    invoke-virtual {v0, v11, v5}, Landroid/content/res/TypedArray;->getDimension(IF)F

    move-result v4

    float-to-int v4, v4

    .line 306
    move v5, v4

    const/4 v4, 0x0

    goto :goto_18a

    .line 303
    :cond_179
    invoke-static {v9}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v4

    if-eqz v4, :cond_185

    .line 304
    const/4 v4, 0x0

    invoke-virtual {v0, v11, v4}, Landroid/content/res/TypedArray;->getColor(II)I

    move-result v5

    goto :goto_18a

    .line 306
    :cond_185
    const/4 v4, 0x0

    invoke-virtual {v0, v11, v4}, Landroid/content/res/TypedArray;->getInt(II)I

    move-result v5

    .line 308
    .local v5, "valueTo":I
    :goto_18a
    const/4 v14, 0x2

    new-array v14, v14, [I

    aput v6, v14, v4

    const/4 v15, 0x1

    aput v5, v14, v15

    invoke-static {v3, v14}, Landroid/animation/PropertyValuesHolder;->ofInt(Ljava/lang/String;[I)Landroid/animation/PropertyValuesHolder;

    move-result-object v14

    goto :goto_1cb

    .line 310
    .end local v5    # "valueTo":I
    :cond_197
    const/4 v4, 0x0

    const/4 v15, 0x1

    new-array v5, v15, [I

    aput v6, v5, v4

    invoke-static {v3, v5}, Landroid/animation/PropertyValuesHolder;->ofInt(Ljava/lang/String;[I)Landroid/animation/PropertyValuesHolder;

    move-result-object v14

    goto :goto_1cb

    .line 313
    .end local v6    # "valueFrom":I
    :cond_1a2
    if-eqz v10, :cond_1c9

    .line 314
    if-ne v9, v4, :cond_1ae

    .line 315
    invoke-virtual {v0, v11, v5}, Landroid/content/res/TypedArray;->getDimension(IF)F

    move-result v4

    float-to-int v4, v4

    .line 319
    move v5, v4

    const/4 v4, 0x0

    goto :goto_1bf

    .line 316
    :cond_1ae
    invoke-static {v9}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v4

    if-eqz v4, :cond_1ba

    .line 317
    const/4 v4, 0x0

    invoke-virtual {v0, v11, v4}, Landroid/content/res/TypedArray;->getColor(II)I

    move-result v5

    goto :goto_1bf

    .line 319
    :cond_1ba
    const/4 v4, 0x0

    invoke-virtual {v0, v11, v4}, Landroid/content/res/TypedArray;->getInt(II)I

    move-result v5

    .line 321
    .restart local v5    # "valueTo":I
    :goto_1bf
    const/4 v6, 0x1

    new-array v6, v6, [I

    aput v5, v6, v4

    invoke-static {v3, v6}, Landroid/animation/PropertyValuesHolder;->ofInt(Ljava/lang/String;[I)Landroid/animation/PropertyValuesHolder;

    move-result-object v14

    goto :goto_1cb

    .line 325
    .end local v5    # "valueTo":I
    :cond_1c9
    move-object/from16 v14, v20

    .end local v20    # "returnValue":Landroid/animation/PropertyValuesHolder;
    .local v14, "returnValue":Landroid/animation/PropertyValuesHolder;
    :goto_1cb
    if-eqz v14, :cond_1d2

    if-eqz v2, :cond_1d2

    .line 326
    invoke-virtual {v14, v2}, Landroid/animation/PropertyValuesHolder;->setEvaluator(Landroid/animation/TypeEvaluator;)V

    .line 330
    .end local v2    # "evaluator":Landroid/animation/TypeEvaluator;
    :cond_1d2
    :goto_1d2
    return-object v14
.end method

.method private static inferValueTypeFromValues(Landroid/content/res/TypedArray;II)I
    .registers 11
    .param p0, "styledAttributes"    # Landroid/content/res/TypedArray;
    .param p1, "valueFromId"    # I
    .param p2, "valueToId"    # I

    .line 654
    invoke-virtual {p0, p1}, Landroid/content/res/TypedArray;->peekValue(I)Landroid/util/TypedValue;

    move-result-object v0

    .line 655
    .local v0, "tvFrom":Landroid/util/TypedValue;
    const/4 v1, 0x1

    const/4 v2, 0x0

    if-eqz v0, :cond_a

    const/4 v3, 0x1

    goto :goto_b

    :cond_a
    const/4 v3, 0x0

    .line 656
    .local v3, "hasFrom":Z
    :goto_b
    if-eqz v3, :cond_10

    iget v4, v0, Landroid/util/TypedValue;->type:I

    goto :goto_11

    :cond_10
    const/4 v4, 0x0

    .line 657
    .local v4, "fromType":I
    :goto_11
    invoke-virtual {p0, p2}, Landroid/content/res/TypedArray;->peekValue(I)Landroid/util/TypedValue;

    move-result-object v5

    .line 658
    .local v5, "tvTo":Landroid/util/TypedValue;
    if-eqz v5, :cond_18

    goto :goto_19

    :cond_18
    const/4 v1, 0x0

    .line 659
    .local v1, "hasTo":Z
    :goto_19
    if-eqz v1, :cond_1e

    iget v6, v5, Landroid/util/TypedValue;->type:I

    goto :goto_1f

    :cond_1e
    const/4 v6, 0x0

    .line 663
    .local v6, "toType":I
    :goto_1f
    if-eqz v3, :cond_27

    invoke-static {v4}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v7

    if-nez v7, :cond_2f

    :cond_27
    if-eqz v1, :cond_31

    invoke-static {v6}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v7

    if-eqz v7, :cond_31

    .line 664
    :cond_2f
    const/4 v2, 0x3

    goto :goto_32

    .line 666
    :cond_31
    nop

    .line 668
    .local v2, "valueType":I
    :goto_32
    return v2
.end method

.method private static inferValueTypeOfKeyframe(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;Lorg/xmlpull/v1/XmlPullParser;)I
    .registers 9
    .param p0, "res"    # Landroid/content/res/Resources;
    .param p1, "theme"    # Landroid/content/res/Resources$Theme;
    .param p2, "attrs"    # Landroid/util/AttributeSet;
    .param p3, "parser"    # Lorg/xmlpull/v1/XmlPullParser;

    .line 635
    sget-object v0, Landroid/support/graphics/drawable/AndroidResources;->STYLEABLE_KEYFRAME:[I

    invoke-static {p0, p1, p2, v0}, Landroid/support/v4/content/res/TypedArrayUtils;->obtainAttributes(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;[I)Landroid/content/res/TypedArray;

    move-result-object v0

    .line 638
    .local v0, "a":Landroid/content/res/TypedArray;
    const-string v1, "value"

    const/4 v2, 0x0

    invoke-static {v0, p3, v1, v2}, Landroid/support/v4/content/res/TypedArrayUtils;->peekNamedValue(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Landroid/util/TypedValue;

    move-result-object v1

    .line 640
    .local v1, "keyframeValue":Landroid/util/TypedValue;
    if-eqz v1, :cond_11

    const/4 v3, 0x1

    goto :goto_12

    :cond_11
    const/4 v3, 0x0

    .line 643
    .local v3, "hasValue":Z
    :goto_12
    if-eqz v3, :cond_1e

    iget v4, v1, Landroid/util/TypedValue;->type:I

    invoke-static {v4}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v4

    if-eqz v4, :cond_1e

    .line 644
    const/4 v2, 0x3

    goto :goto_1f

    .line 646
    :cond_1e
    nop

    .line 648
    .local v2, "valueType":I
    :goto_1f
    invoke-virtual {v0}, Landroid/content/res/TypedArray;->recycle()V

    .line 649
    return v2
.end method

.method private static isColorType(I)Z
    .registers 2
    .param p0, "type"    # I

    .line 916
    const/16 v0, 0x1c

    if-lt p0, v0, :cond_a

    const/16 v0, 0x1f

    if-gt p0, v0, :cond_a

    const/4 v0, 0x1

    goto :goto_b

    :cond_a
    const/4 v0, 0x0

    :goto_b
    return v0
.end method

.method public static loadAnimator(Landroid/content/Context;I)Landroid/animation/Animator;
    .registers 4
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "id"    # I
        .annotation build Landroid/support/annotation/AnimatorRes;
        .end annotation
    .end param
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Landroid/content/res/Resources$NotFoundException;
        }
    .end annotation

    .line 99
    sget v0, Landroid/os/Build$VERSION;->SDK_INT:I

    const/16 v1, 0x18

    if-lt v0, v1, :cond_b

    .line 100
    invoke-static {p0, p1}, Landroid/animation/AnimatorInflater;->loadAnimator(Landroid/content/Context;I)Landroid/animation/Animator;

    move-result-object v0

    goto :goto_17

    .line 102
    :cond_b
    invoke-virtual {p0}, Landroid/content/Context;->getResources()Landroid/content/res/Resources;

    move-result-object v0

    invoke-virtual {p0}, Landroid/content/Context;->getTheme()Landroid/content/res/Resources$Theme;

    move-result-object v1

    invoke-static {p0, v0, v1, p1}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;I)Landroid/animation/Animator;

    move-result-object v0

    .line 104
    .local v0, "objectAnimator":Landroid/animation/Animator;
    :goto_17
    return-object v0
.end method

.method public static loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;I)Landroid/animation/Animator;
    .registers 5
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "resources"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "id"    # I
        .annotation build Landroid/support/annotation/AnimatorRes;
        .end annotation
    .end param
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Landroid/content/res/Resources$NotFoundException;
        }
    .end annotation

    .line 118
    const/high16 v0, 0x3f800000    # 1.0f

    invoke-static {p0, p1, p2, p3, v0}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;IF)Landroid/animation/Animator;

    move-result-object v0

    return-object v0
.end method

.method public static loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;IF)Landroid/animation/Animator;
    .registers 10
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "resources"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "id"    # I
        .annotation build Landroid/support/annotation/AnimatorRes;
        .end annotation
    .end param
    .param p4, "pathErrorScale"    # F
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Landroid/content/res/Resources$NotFoundException;
        }
    .end annotation

    .line 128
    const/4 v0, 0x0

    .line 130
    .local v0, "parser":Landroid/content/res/XmlResourceParser;
    :try_start_1
    invoke-virtual {p1, p3}, Landroid/content/res/Resources;->getAnimation(I)Landroid/content/res/XmlResourceParser;

    move-result-object v1

    move-object v0, v1

    .line 131
    invoke-static {p0, p1, p2, v0, p4}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->createAnimatorFromXml(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;F)Landroid/animation/Animator;

    move-result-object v1
    :try_end_a
    .catch Lorg/xmlpull/v1/XmlPullParserException; {:try_start_1 .. :try_end_a} :catch_32
    .catch Ljava/io/IOException; {:try_start_1 .. :try_end_a} :catch_13
    .catchall {:try_start_1 .. :try_end_a} :catchall_11

    .line 132
    .local v1, "animator":Landroid/animation/Animator;
    nop

    .line 146
    if-eqz v0, :cond_10

    invoke-interface {v0}, Landroid/content/res/XmlResourceParser;->close()V

    :cond_10
    return-object v1

    .end local v1    # "animator":Landroid/animation/Animator;
    :catchall_11
    move-exception v1

    goto :goto_51

    .line 139
    :catch_13
    move-exception v1

    .line 140
    .local v1, "ex":Ljava/io/IOException;
    :try_start_14
    new-instance v2, Landroid/content/res/Resources$NotFoundException;

    new-instance v3, Ljava/lang/StringBuilder;

    invoke-direct {v3}, Ljava/lang/StringBuilder;-><init>()V

    const-string v4, "Can\'t load animation resource ID #0x"

    invoke-virtual {v3, v4}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 142
    invoke-static {p3}, Ljava/lang/Integer;->toHexString(I)Ljava/lang/String;

    move-result-object v4

    invoke-virtual {v3, v4}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v3}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v3

    invoke-direct {v2, v3}, Landroid/content/res/Resources$NotFoundException;-><init>(Ljava/lang/String;)V

    .line 143
    .local v2, "rnf":Landroid/content/res/Resources$NotFoundException;
    invoke-virtual {v2, v1}, Landroid/content/res/Resources$NotFoundException;->initCause(Ljava/lang/Throwable;)Ljava/lang/Throwable;

    .line 144
    throw v2

    .line 133
    .end local v1    # "ex":Ljava/io/IOException;
    .end local v2    # "rnf":Landroid/content/res/Resources$NotFoundException;
    :catch_32
    move-exception v1

    .line 134
    .local v1, "ex":Lorg/xmlpull/v1/XmlPullParserException;
    new-instance v2, Landroid/content/res/Resources$NotFoundException;

    new-instance v3, Ljava/lang/StringBuilder;

    invoke-direct {v3}, Ljava/lang/StringBuilder;-><init>()V

    const-string v4, "Can\'t load animation resource ID #0x"

    invoke-virtual {v3, v4}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 136
    invoke-static {p3}, Ljava/lang/Integer;->toHexString(I)Ljava/lang/String;

    move-result-object v4

    invoke-virtual {v3, v4}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v3}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v3

    invoke-direct {v2, v3}, Landroid/content/res/Resources$NotFoundException;-><init>(Ljava/lang/String;)V

    .line 137
    .restart local v2    # "rnf":Landroid/content/res/Resources$NotFoundException;
    invoke-virtual {v2, v1}, Landroid/content/res/Resources$NotFoundException;->initCause(Ljava/lang/Throwable;)Ljava/lang/Throwable;

    .line 138
    throw v2
    :try_end_51
    .catchall {:try_start_14 .. :try_end_51} :catchall_11

    .line 146
    .end local v1    # "ex":Lorg/xmlpull/v1/XmlPullParserException;
    .end local v2    # "rnf":Landroid/content/res/Resources$NotFoundException;
    :goto_51
    if-eqz v0, :cond_56

    invoke-interface {v0}, Landroid/content/res/XmlResourceParser;->close()V

    :cond_56
    throw v1
.end method

.method private static loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;Landroid/animation/ValueAnimator;FLorg/xmlpull/v1/XmlPullParser;)Landroid/animation/ValueAnimator;
    .registers 11
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "attrs"    # Landroid/util/AttributeSet;
    .param p4, "anim"    # Landroid/animation/ValueAnimator;
    .param p5, "pathErrorScale"    # F
    .param p6, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Landroid/content/res/Resources$NotFoundException;
        }
    .end annotation

    .line 889
    sget-object v0, Landroid/support/graphics/drawable/AndroidResources;->STYLEABLE_ANIMATOR:[I

    invoke-static {p1, p2, p3, v0}, Landroid/support/v4/content/res/TypedArrayUtils;->obtainAttributes(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;[I)Landroid/content/res/TypedArray;

    move-result-object v0

    .line 891
    .local v0, "arrayAnimator":Landroid/content/res/TypedArray;
    sget-object v1, Landroid/support/graphics/drawable/AndroidResources;->STYLEABLE_PROPERTY_ANIMATOR:[I

    invoke-static {p1, p2, p3, v1}, Landroid/support/v4/content/res/TypedArrayUtils;->obtainAttributes(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;[I)Landroid/content/res/TypedArray;

    move-result-object v1

    .line 894
    .local v1, "arrayObjectAnimator":Landroid/content/res/TypedArray;
    if-nez p4, :cond_14

    .line 895
    new-instance v2, Landroid/animation/ValueAnimator;

    invoke-direct {v2}, Landroid/animation/ValueAnimator;-><init>()V

    move-object p4, v2

    .line 898
    :cond_14
    invoke-static {p4, v0, v1, p5, p6}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->parseAnimatorFromTypeArray(Landroid/animation/ValueAnimator;Landroid/content/res/TypedArray;Landroid/content/res/TypedArray;FLorg/xmlpull/v1/XmlPullParser;)V

    .line 901
    const-string v2, "interpolator"

    const/4 v3, 0x0

    invoke-static {v0, p6, v2, v3, v3}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedResourceId(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v2

    .line 903
    .local v2, "resID":I
    if-lez v2, :cond_27

    .line 904
    invoke-static {p0, v2}, Landroid/support/graphics/drawable/AnimationUtilsCompat;->loadInterpolator(Landroid/content/Context;I)Landroid/view/animation/Interpolator;

    move-result-object v3

    .line 905
    .local v3, "interpolator":Landroid/view/animation/Interpolator;
    invoke-virtual {p4, v3}, Landroid/animation/ValueAnimator;->setInterpolator(Landroid/animation/TimeInterpolator;)V

    .line 908
    .end local v3    # "interpolator":Landroid/view/animation/Interpolator;
    :cond_27
    invoke-virtual {v0}, Landroid/content/res/TypedArray;->recycle()V

    .line 909
    if-eqz v1, :cond_2f

    .line 910
    invoke-virtual {v1}, Landroid/content/res/TypedArray;->recycle()V

    .line 912
    :cond_2f
    return-object p4
.end method

.method private static loadKeyframe(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;ILorg/xmlpull/v1/XmlPullParser;)Landroid/animation/Keyframe;
    .registers 15
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "attrs"    # Landroid/util/AttributeSet;
    .param p4, "valueType"    # I
    .param p5, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/xmlpull/v1/XmlPullParserException;,
            Ljava/io/IOException;
        }
    .end annotation

    .line 817
    sget-object v0, Landroid/support/graphics/drawable/AndroidResources;->STYLEABLE_KEYFRAME:[I

    invoke-static {p1, p2, p3, v0}, Landroid/support/v4/content/res/TypedArrayUtils;->obtainAttributes(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;[I)Landroid/content/res/TypedArray;

    move-result-object v0

    .line 820
    .local v0, "a":Landroid/content/res/TypedArray;
    const/4 v1, 0x0

    .line 822
    .local v1, "keyframe":Landroid/animation/Keyframe;
    const-string v2, "fraction"

    const/4 v3, 0x3

    const/high16 v4, -0x40800000    # -1.0f

    invoke-static {v0, p5, v2, v3, v4}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedFloat(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;IF)F

    move-result v2

    .line 825
    .local v2, "fraction":F
    const-string v4, "value"

    const/4 v5, 0x0

    invoke-static {v0, p5, v4, v5}, Landroid/support/v4/content/res/TypedArrayUtils;->peekNamedValue(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Landroid/util/TypedValue;

    move-result-object v4

    .line 827
    .local v4, "keyframeValue":Landroid/util/TypedValue;
    const/4 v6, 0x1

    if-eqz v4, :cond_1c

    const/4 v7, 0x1

    goto :goto_1d

    :cond_1c
    const/4 v7, 0x0

    .line 828
    .local v7, "hasValue":Z
    :goto_1d
    const/4 v8, 0x4

    if-ne p4, v8, :cond_2d

    .line 831
    if-eqz v7, :cond_2c

    iget v8, v4, Landroid/util/TypedValue;->type:I

    invoke-static {v8}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->isColorType(I)Z

    move-result v8

    if-eqz v8, :cond_2c

    .line 832
    const/4 p4, 0x3

    goto :goto_2d

    .line 834
    :cond_2c
    const/4 p4, 0x0

    .line 838
    :cond_2d
    :goto_2d
    if-eqz v7, :cond_4c

    .line 839
    if-eq p4, v3, :cond_41

    packed-switch p4, :pswitch_data_6c

    goto :goto_4b

    .line 841
    :pswitch_35
    const-string v3, "value"

    const/4 v8, 0x0

    invoke-static {v0, p5, v3, v5, v8}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedFloat(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;IF)F

    move-result v3

    .line 843
    .local v3, "value":F
    invoke-static {v2, v3}, Landroid/animation/Keyframe;->ofFloat(FF)Landroid/animation/Keyframe;

    move-result-object v1

    .line 844
    goto :goto_4b

    .line 847
    .end local v3    # "value":F
    :cond_41
    :pswitch_41
    const-string v3, "value"

    invoke-static {v0, p5, v3, v5, v5}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v3

    .line 849
    .local v3, "intValue":I
    invoke-static {v2, v3}, Landroid/animation/Keyframe;->ofInt(FI)Landroid/animation/Keyframe;

    move-result-object v1

    .line 850
    .end local v3    # "intValue":I
    :goto_4b
    goto :goto_58

    .line 853
    :cond_4c
    if-nez p4, :cond_53

    invoke-static {v2}, Landroid/animation/Keyframe;->ofFloat(F)Landroid/animation/Keyframe;

    move-result-object v3

    goto :goto_57

    .line 854
    :cond_53
    invoke-static {v2}, Landroid/animation/Keyframe;->ofInt(F)Landroid/animation/Keyframe;

    move-result-object v3

    :goto_57
    move-object v1, v3

    .line 857
    :goto_58
    const-string v3, "interpolator"

    invoke-static {v0, p5, v3, v6, v5}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedResourceId(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v3

    .line 859
    .local v3, "resID":I
    if-lez v3, :cond_67

    .line 860
    invoke-static {p0, v3}, Landroid/support/graphics/drawable/AnimationUtilsCompat;->loadInterpolator(Landroid/content/Context;I)Landroid/view/animation/Interpolator;

    move-result-object v5

    .line 861
    .local v5, "interpolator":Landroid/view/animation/Interpolator;
    invoke-virtual {v1, v5}, Landroid/animation/Keyframe;->setInterpolator(Landroid/animation/TimeInterpolator;)V

    .line 863
    .end local v5    # "interpolator":Landroid/view/animation/Interpolator;
    :cond_67
    invoke-virtual {v0}, Landroid/content/res/TypedArray;->recycle()V

    .line 865
    return-object v1

    nop

    :pswitch_data_6c
    .packed-switch 0x0
        :pswitch_35
        :pswitch_41
    .end packed-switch
.end method

.method private static loadObjectAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;FLorg/xmlpull/v1/XmlPullParser;)Landroid/animation/ObjectAnimator;
    .registers 14
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "attrs"    # Landroid/util/AttributeSet;
    .param p4, "pathErrorScale"    # F
    .param p5, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Landroid/content/res/Resources$NotFoundException;
        }
    .end annotation

    .line 871
    new-instance v0, Landroid/animation/ObjectAnimator;

    invoke-direct {v0}, Landroid/animation/ObjectAnimator;-><init>()V

    .line 873
    .local v0, "anim":Landroid/animation/ObjectAnimator;
    move-object v1, p0

    move-object v2, p1

    move-object v3, p2

    move-object v4, p3

    move-object v5, v0

    move v6, p4

    move-object v7, p5

    invoke-static/range {v1 .. v7}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadAnimator(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;Landroid/animation/ValueAnimator;FLorg/xmlpull/v1/XmlPullParser;)Landroid/animation/ValueAnimator;

    .line 875
    return-object v0
.end method

.method private static loadPvh(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Landroid/animation/PropertyValuesHolder;
    .registers 29
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .param p4, "propertyName"    # Ljava/lang/String;
    .param p5, "valueType"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/xmlpull/v1/XmlPullParserException;,
            Ljava/io/IOException;
        }
    .end annotation

    .line 691
    const/4 v0, 0x0

    .line 692
    .local v0, "value":Landroid/animation/PropertyValuesHolder;
    const/4 v1, 0x0

    move/from16 v2, p5

    .line 695
    .end local p5    # "valueType":I
    .local v1, "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .local v2, "valueType":I
    :goto_4
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->next()I

    move-result v3

    move v4, v3

    .local v4, "type":I
    const/4 v5, 0x3

    if-eq v3, v5, :cond_5c

    const/4 v3, 0x1

    if-eq v4, v3, :cond_5c

    .line 697
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getName()Ljava/lang/String;

    move-result-object v3

    .line 698
    .local v3, "name":Ljava/lang/String;
    const-string v5, "keyframe"

    invoke-virtual {v3, v5}, Ljava/lang/String;->equals(Ljava/lang/Object;)Z

    move-result v5

    if-eqz v5, :cond_55

    .line 699
    const/4 v5, 0x4

    if-ne v2, v5, :cond_2d

    .line 700
    invoke-static/range {p3 .. p3}, Landroid/util/Xml;->asAttributeSet(Lorg/xmlpull/v1/XmlPullParser;)Landroid/util/AttributeSet;

    move-result-object v5

    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p3

    invoke-static {v12, v13, v5, v14}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->inferValueTypeOfKeyframe(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;Lorg/xmlpull/v1/XmlPullParser;)I

    move-result v2

    goto :goto_33

    .line 703
    :cond_2d
    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p3

    :goto_33
    invoke-static/range {p3 .. p3}, Landroid/util/Xml;->asAttributeSet(Lorg/xmlpull/v1/XmlPullParser;)Landroid/util/AttributeSet;

    move-result-object v9

    move-object/from16 v6, p0

    move-object/from16 v7, p1

    move-object/from16 v8, p2

    move v10, v2

    move-object/from16 v11, p3

    invoke-static/range {v6 .. v11}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadKeyframe(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;ILorg/xmlpull/v1/XmlPullParser;)Landroid/animation/Keyframe;

    move-result-object v5

    .line 705
    .local v5, "keyframe":Landroid/animation/Keyframe;
    if-eqz v5, :cond_51

    .line 706
    if-nez v1, :cond_4e

    .line 707
    new-instance v6, Ljava/util/ArrayList;

    invoke-direct {v6}, Ljava/util/ArrayList;-><init>()V

    move-object v1, v6

    .line 709
    :cond_4e
    invoke-virtual {v1, v5}, Ljava/util/ArrayList;->add(Ljava/lang/Object;)Z

    .line 711
    :cond_51
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->next()I

    goto :goto_5b

    .line 713
    .end local v3    # "name":Ljava/lang/String;
    .end local v5    # "keyframe":Landroid/animation/Keyframe;
    :cond_55
    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p3

    :goto_5b
    goto :goto_4

    .line 716
    :cond_5c
    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p3

    if-eqz v1, :cond_14e

    invoke-virtual {v1}, Ljava/util/ArrayList;->size()I

    move-result v3

    move v6, v3

    .local v6, "count":I
    if-lez v3, :cond_14e

    .line 722
    const/4 v3, 0x0

    invoke-virtual {v1, v3}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v7

    check-cast v7, Landroid/animation/Keyframe;

    .line 723
    .local v7, "firstKeyframe":Landroid/animation/Keyframe;
    add-int/lit8 v8, v6, -0x1

    invoke-virtual {v1, v8}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v8

    check-cast v8, Landroid/animation/Keyframe;

    .line 724
    .local v8, "lastKeyframe":Landroid/animation/Keyframe;
    invoke-virtual {v8}, Landroid/animation/Keyframe;->getFraction()F

    move-result v9

    .line 725
    .local v9, "endFraction":F
    const/high16 v10, 0x3f800000    # 1.0f

    const/4 v11, 0x0

    cmpg-float v15, v9, v10

    if-gez v15, :cond_9a

    .line 726
    cmpg-float v15, v9, v11

    if-gez v15, :cond_8d

    .line 727
    invoke-virtual {v8, v10}, Landroid/animation/Keyframe;->setFraction(F)V

    goto :goto_9a

    .line 729
    :cond_8d
    invoke-virtual {v1}, Ljava/util/ArrayList;->size()I

    move-result v15

    invoke-static {v8, v10}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->createNewKeyframe(Landroid/animation/Keyframe;F)Landroid/animation/Keyframe;

    move-result-object v5

    invoke-virtual {v1, v15, v5}, Ljava/util/ArrayList;->add(ILjava/lang/Object;)V

    .line 730
    add-int/lit8 v6, v6, 0x1

    .line 733
    :cond_9a
    :goto_9a
    invoke-virtual {v7}, Landroid/animation/Keyframe;->getFraction()F

    move-result v5

    .line 734
    .local v5, "startFraction":F
    cmpl-float v15, v5, v11

    if-eqz v15, :cond_b3

    .line 735
    cmpg-float v15, v5, v11

    if-gez v15, :cond_aa

    .line 736
    invoke-virtual {v7, v11}, Landroid/animation/Keyframe;->setFraction(F)V

    goto :goto_b3

    .line 738
    :cond_aa
    invoke-static {v7, v11}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->createNewKeyframe(Landroid/animation/Keyframe;F)Landroid/animation/Keyframe;

    move-result-object v15

    invoke-virtual {v1, v3, v15}, Ljava/util/ArrayList;->add(ILjava/lang/Object;)V

    .line 739
    add-int/lit8 v6, v6, 0x1

    .line 742
    :cond_b3
    :goto_b3
    new-array v15, v6, [Landroid/animation/Keyframe;

    .line 743
    .local v15, "keyframeArray":[Landroid/animation/Keyframe;
    invoke-virtual {v1, v15}, Ljava/util/ArrayList;->toArray([Ljava/lang/Object;)[Ljava/lang/Object;

    .line 744
    nop

    .local v3, "i":I
    :goto_b9
    if-ge v3, v6, :cond_137

    .line 745
    aget-object v10, v15, v3

    .line 746
    .local v10, "keyframe":Landroid/animation/Keyframe;
    invoke-virtual {v10}, Landroid/animation/Keyframe;->getFraction()F

    move-result v17

    cmpg-float v17, v17, v11

    if-gez v17, :cond_123

    .line 747
    if-nez v3, :cond_d3

    .line 748
    invoke-virtual {v10, v11}, Landroid/animation/Keyframe;->setFraction(F)V

    .line 744
    .end local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v4    # "type":I
    .end local v10    # "keyframe":Landroid/animation/Keyframe;
    .local v19, "value":Landroid/animation/PropertyValuesHolder;
    .local v21, "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .local v22, "type":I
    :goto_ca
    move-object/from16 v19, v0

    move-object/from16 v21, v1

    move/from16 v22, v4

    const/16 v16, 0x0

    goto :goto_12b

    .line 749
    .end local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v22    # "type":I
    .restart local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .restart local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v4    # "type":I
    .restart local v10    # "keyframe":Landroid/animation/Keyframe;
    :cond_d3
    add-int/lit8 v11, v6, -0x1

    if-ne v3, v11, :cond_dd

    .line 750
    const/high16 v11, 0x3f800000    # 1.0f

    invoke-virtual {v10, v11}, Landroid/animation/Keyframe;->setFraction(F)V

    goto :goto_ca

    .line 754
    :cond_dd
    const/high16 v11, 0x3f800000    # 1.0f

    move/from16 v18, v3

    .line 755
    .local v18, "startIndex":I
    move/from16 v16, v3

    .line 756
    .local v16, "endIndex":I
    move/from16 v11, v18

    .end local v18    # "startIndex":I
    .local v11, "startIndex":I
    add-int/lit8 v18, v11, 0x1

    move-object/from16 v19, v0

    move/from16 v0, v16

    .end local v16    # "endIndex":I
    .local v0, "endIndex":I
    .local v18, "j":I
    .restart local v19    # "value":Landroid/animation/PropertyValuesHolder;
    :goto_eb
    move/from16 v20, v18

    .end local v18    # "j":I
    .local v20, "j":I
    move-object/from16 v21, v1

    .end local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    add-int/lit8 v1, v6, -0x1

    move/from16 v22, v4

    move/from16 v4, v20

    .end local v20    # "j":I
    .local v4, "j":I
    .restart local v22    # "type":I
    if-ge v4, v1, :cond_10c

    .line 757
    aget-object v1, v15, v4

    invoke-virtual {v1}, Landroid/animation/Keyframe;->getFraction()F

    move-result v1

    const/16 v16, 0x0

    cmpl-float v1, v1, v16

    if-ltz v1, :cond_104

    .line 758
    goto :goto_10e

    .line 760
    :cond_104
    move v0, v4

    .line 756
    add-int/lit8 v18, v4, 0x1

    move-object/from16 v1, v21

    move/from16 v4, v22

    goto :goto_eb

    .line 762
    .end local v4    # "j":I
    :cond_10c
    const/16 v16, 0x0

    :goto_10e
    add-int/lit8 v1, v0, 0x1

    aget-object v1, v15, v1

    invoke-virtual {v1}, Landroid/animation/Keyframe;->getFraction()F

    move-result v1

    add-int/lit8 v18, v11, -0x1

    aget-object v4, v15, v18

    .line 763
    invoke-virtual {v4}, Landroid/animation/Keyframe;->getFraction()F

    move-result v4

    sub-float/2addr v1, v4

    .line 764
    .local v1, "gap":F
    invoke-static {v15, v1, v11, v0}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->distributeKeyframes([Landroid/animation/Keyframe;FII)V

    goto :goto_12b

    .line 744
    .end local v10    # "keyframe":Landroid/animation/Keyframe;
    .end local v11    # "startIndex":I
    .end local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v22    # "type":I
    .local v0, "value":Landroid/animation/PropertyValuesHolder;
    .local v1, "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .local v4, "type":I
    :cond_123
    move-object/from16 v19, v0

    move-object/from16 v21, v1

    move/from16 v22, v4

    const/16 v16, 0x0

    .end local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v4    # "type":I
    .restart local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .restart local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v22    # "type":I
    :goto_12b
    add-int/lit8 v3, v3, 0x1

    move-object/from16 v0, v19

    move-object/from16 v1, v21

    move/from16 v4, v22

    const/high16 v10, 0x3f800000    # 1.0f

    const/4 v11, 0x0

    goto :goto_b9

    .line 768
    .end local v3    # "i":I
    .end local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v22    # "type":I
    .restart local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .restart local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v4    # "type":I
    :cond_137
    move-object/from16 v19, v0

    move-object/from16 v21, v1

    move/from16 v22, v4

    .end local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v4    # "type":I
    .restart local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .restart local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v22    # "type":I
    move-object/from16 v0, p4

    invoke-static {v0, v15}, Landroid/animation/PropertyValuesHolder;->ofKeyframe(Ljava/lang/String;[Landroid/animation/Keyframe;)Landroid/animation/PropertyValuesHolder;

    move-result-object v1

    .line 769
    .end local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .local v1, "value":Landroid/animation/PropertyValuesHolder;
    const/4 v3, 0x3

    if-ne v2, v3, :cond_158

    .line 770
    invoke-static {}, Landroid/support/graphics/drawable/ArgbEvaluator;->getInstance()Landroid/support/graphics/drawable/ArgbEvaluator;

    move-result-object v3

    invoke-virtual {v1, v3}, Landroid/animation/PropertyValuesHolder;->setEvaluator(Landroid/animation/TypeEvaluator;)V

    goto :goto_158

    .line 774
    .end local v5    # "startFraction":F
    .end local v6    # "count":I
    .end local v7    # "firstKeyframe":Landroid/animation/Keyframe;
    .end local v8    # "lastKeyframe":Landroid/animation/Keyframe;
    .end local v9    # "endFraction":F
    .end local v15    # "keyframeArray":[Landroid/animation/Keyframe;
    .end local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v22    # "type":I
    .restart local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .local v1, "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v4    # "type":I
    :cond_14e
    move-object/from16 v19, v0

    move-object/from16 v21, v1

    move/from16 v22, v4

    move-object/from16 v0, p4

    .end local v0    # "value":Landroid/animation/PropertyValuesHolder;
    .end local v1    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .end local v4    # "type":I
    .restart local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .restart local v21    # "keyframes":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/Keyframe;>;"
    .restart local v22    # "type":I
    move-object/from16 v1, v19

    .end local v19    # "value":Landroid/animation/PropertyValuesHolder;
    .local v1, "value":Landroid/animation/PropertyValuesHolder;
    :cond_158
    :goto_158
    return-object v1
.end method

.method private static loadValues(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Landroid/util/AttributeSet;)[Landroid/animation/PropertyValuesHolder;
    .registers 22
    .param p0, "context"    # Landroid/content/Context;
    .param p1, "res"    # Landroid/content/res/Resources;
    .param p2, "theme"    # Landroid/content/res/Resources$Theme;
    .param p3, "parser"    # Lorg/xmlpull/v1/XmlPullParser;
    .param p4, "attrs"    # Landroid/util/AttributeSet;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/xmlpull/v1/XmlPullParserException;,
            Ljava/io/IOException;
        }
    .end annotation

    .line 575
    move-object/from16 v6, p3

    const/4 v0, 0x0

    .local v0, "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    :goto_3
    move-object v7, v0

    .line 578
    .end local v0    # "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    .local v7, "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getEventType()I

    move-result v0

    move v8, v0

    .local v8, "type":I
    const/4 v1, 0x3

    if-eq v0, v1, :cond_77

    const/4 v10, 0x1

    if-eq v8, v10, :cond_77

    .line 581
    const/4 v0, 0x2

    if-eq v8, v0, :cond_18

    .line 582
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->next()I

    .line 583
    nop

    .line 575
    move-object v0, v7

    goto :goto_3

    .line 586
    :cond_18
    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->getName()Ljava/lang/String;

    move-result-object v11

    .line 588
    .local v11, "name":Ljava/lang/String;
    const-string v2, "propertyValuesHolder"

    invoke-virtual {v11, v2}, Ljava/lang/String;->equals(Ljava/lang/Object;)Z

    move-result v2

    if-eqz v2, :cond_6c

    .line 589
    sget-object v2, Landroid/support/graphics/drawable/AndroidResources;->STYLEABLE_PROPERTY_VALUES_HOLDER:[I

    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p4

    invoke-static {v12, v13, v14, v2}, Landroid/support/v4/content/res/TypedArrayUtils;->obtainAttributes(Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Landroid/util/AttributeSet;[I)Landroid/content/res/TypedArray;

    move-result-object v15

    .line 592
    .local v15, "a":Landroid/content/res/TypedArray;
    const-string v2, "propertyName"

    invoke-static {v15, v6, v2, v1}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedString(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Ljava/lang/String;

    move-result-object v5

    .line 594
    .local v5, "propertyName":Ljava/lang/String;
    const-string v1, "valueType"

    const/4 v2, 0x4

    invoke-static {v15, v6, v1, v0, v2}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v4

    .line 598
    .local v4, "valueType":I
    move-object/from16 v0, p0

    move-object/from16 v1, p1

    move-object/from16 v2, p2

    move-object/from16 v3, p3

    move/from16 v16, v4

    .end local v4    # "valueType":I
    .local v16, "valueType":I
    move-object v4, v5

    move-object v9, v5

    .end local v5    # "propertyName":Ljava/lang/String;
    .local v9, "propertyName":Ljava/lang/String;
    move/from16 v5, v16

    invoke-static/range {v0 .. v5}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->loadPvh(Landroid/content/Context;Landroid/content/res/Resources;Landroid/content/res/Resources$Theme;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Landroid/animation/PropertyValuesHolder;

    move-result-object v0

    .line 600
    .local v0, "pvh":Landroid/animation/PropertyValuesHolder;
    if-nez v0, :cond_59

    .line 601
    move/from16 v1, v16

    const/4 v2, 0x0

    .end local v16    # "valueType":I
    .local v1, "valueType":I
    invoke-static {v15, v1, v2, v10, v9}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->getPVH(Landroid/content/res/TypedArray;IIILjava/lang/String;)Landroid/animation/PropertyValuesHolder;

    move-result-object v0

    goto :goto_5b

    .line 606
    .end local v1    # "valueType":I
    .restart local v16    # "valueType":I
    :cond_59
    move/from16 v1, v16

    .end local v16    # "valueType":I
    .restart local v1    # "valueType":I
    :goto_5b
    if-eqz v0, :cond_68

    .line 607
    if-nez v7, :cond_65

    .line 608
    new-instance v2, Ljava/util/ArrayList;

    invoke-direct {v2}, Ljava/util/ArrayList;-><init>()V

    move-object v7, v2

    .line 610
    :cond_65
    invoke-virtual {v7, v0}, Ljava/util/ArrayList;->add(Ljava/lang/Object;)Z

    .line 612
    :cond_68
    invoke-virtual {v15}, Landroid/content/res/TypedArray;->recycle()V

    goto :goto_72

    .line 615
    .end local v0    # "pvh":Landroid/animation/PropertyValuesHolder;
    .end local v1    # "valueType":I
    .end local v9    # "propertyName":Ljava/lang/String;
    .end local v15    # "a":Landroid/content/res/TypedArray;
    :cond_6c
    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p4

    .end local v7    # "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    .local v0, "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    :goto_72
    move-object v0, v7

    invoke-interface/range {p3 .. p3}, Lorg/xmlpull/v1/XmlPullParser;->next()I

    .line 616
    .end local v11    # "name":Ljava/lang/String;
    goto :goto_3

    .line 618
    .end local v0    # "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    .restart local v7    # "values":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/animation/PropertyValuesHolder;>;"
    :cond_77
    move-object/from16 v12, p1

    move-object/from16 v13, p2

    move-object/from16 v14, p4

    const/4 v2, 0x0

    const/4 v0, 0x0

    .line 619
    .local v0, "valuesArray":[Landroid/animation/PropertyValuesHolder;
    if-eqz v7, :cond_95

    .line 620
    invoke-virtual {v7}, Ljava/util/ArrayList;->size()I

    move-result v1

    .line 621
    .local v1, "count":I
    new-array v0, v1, [Landroid/animation/PropertyValuesHolder;

    .line 622
    nop

    .local v2, "i":I
    :goto_88
    if-ge v2, v1, :cond_95

    .line 623
    invoke-virtual {v7, v2}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v3

    check-cast v3, Landroid/animation/PropertyValuesHolder;

    aput-object v3, v0, v2

    .line 622
    add-int/lit8 v2, v2, 0x1

    goto :goto_88

    .line 626
    .end local v1    # "count":I
    .end local v2    # "i":I
    :cond_95
    return-object v0
.end method

.method private static parseAnimatorFromTypeArray(Landroid/animation/ValueAnimator;Landroid/content/res/TypedArray;Landroid/content/res/TypedArray;FLorg/xmlpull/v1/XmlPullParser;)V
    .registers 16
    .param p0, "anim"    # Landroid/animation/ValueAnimator;
    .param p1, "arrayAnimator"    # Landroid/content/res/TypedArray;
    .param p2, "arrayObjectAnimator"    # Landroid/content/res/TypedArray;
    .param p3, "pixelSize"    # F
    .param p4, "parser"    # Lorg/xmlpull/v1/XmlPullParser;

    .line 344
    const-string v0, "duration"

    const/4 v1, 0x1

    const/16 v2, 0x12c

    invoke-static {p1, p4, v0, v1, v2}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v0

    int-to-long v2, v0

    .line 346
    .local v2, "duration":J
    const-string v0, "startOffset"

    const/4 v4, 0x0

    const/4 v5, 0x2

    invoke-static {p1, p4, v0, v5, v4}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v0

    int-to-long v5, v0

    .line 348
    .local v5, "startDelay":J
    const-string v0, "valueType"

    const/4 v7, 0x4

    const/4 v8, 0x7

    invoke-static {p1, p4, v0, v8, v7}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v0

    .line 352
    .local v0, "valueType":I
    const-string v8, "valueFrom"

    invoke-static {p4, v8}, Landroid/support/v4/content/res/TypedArrayUtils;->hasAttribute(Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;)Z

    move-result v8

    if-eqz v8, :cond_42

    const-string v8, "valueTo"

    .line 353
    invoke-static {p4, v8}, Landroid/support/v4/content/res/TypedArrayUtils;->hasAttribute(Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;)Z

    move-result v8

    if-eqz v8, :cond_42

    .line 354
    const/4 v8, 0x6

    const/4 v9, 0x5

    if-ne v0, v7, :cond_33

    .line 355
    invoke-static {p1, v9, v8}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->inferValueTypeFromValues(Landroid/content/res/TypedArray;II)I

    move-result v0

    .line 359
    :cond_33
    const-string v10, ""

    invoke-static {p1, v0, v9, v8, v10}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->getPVH(Landroid/content/res/TypedArray;IIILjava/lang/String;)Landroid/animation/PropertyValuesHolder;

    move-result-object v8

    .line 362
    .local v8, "pvh":Landroid/animation/PropertyValuesHolder;
    if-eqz v8, :cond_42

    .line 363
    new-array v9, v1, [Landroid/animation/PropertyValuesHolder;

    aput-object v8, v9, v4

    invoke-virtual {p0, v9}, Landroid/animation/ValueAnimator;->setValues([Landroid/animation/PropertyValuesHolder;)V

    .line 366
    .end local v8    # "pvh":Landroid/animation/PropertyValuesHolder;
    :cond_42
    invoke-virtual {p0, v2, v3}, Landroid/animation/ValueAnimator;->setDuration(J)Landroid/animation/ValueAnimator;

    .line 367
    invoke-virtual {p0, v5, v6}, Landroid/animation/ValueAnimator;->setStartDelay(J)V

    .line 369
    const-string v8, "repeatCount"

    const/4 v9, 0x3

    invoke-static {p1, p4, v8, v9, v4}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v4

    invoke-virtual {p0, v4}, Landroid/animation/ValueAnimator;->setRepeatCount(I)V

    .line 371
    const-string v4, "repeatMode"

    invoke-static {p1, p4, v4, v7, v1}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedInt(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;II)I

    move-result v1

    invoke-virtual {p0, v1}, Landroid/animation/ValueAnimator;->setRepeatMode(I)V

    .line 374
    if-eqz p2, :cond_60

    .line 375
    invoke-static {p0, p2, v0, p3, p4}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->setupObjectAnimator(Landroid/animation/ValueAnimator;Landroid/content/res/TypedArray;IFLorg/xmlpull/v1/XmlPullParser;)V

    .line 377
    :cond_60
    return-void
.end method

.method private static setupObjectAnimator(Landroid/animation/ValueAnimator;Landroid/content/res/TypedArray;IFLorg/xmlpull/v1/XmlPullParser;)V
    .registers 12
    .param p0, "anim"    # Landroid/animation/ValueAnimator;
    .param p1, "arrayObjectAnimator"    # Landroid/content/res/TypedArray;
    .param p2, "valueType"    # I
    .param p3, "pixelSize"    # F
    .param p4, "parser"    # Lorg/xmlpull/v1/XmlPullParser;

    .line 389
    move-object v0, p0

    check-cast v0, Landroid/animation/ObjectAnimator;

    .line 390
    .local v0, "oa":Landroid/animation/ObjectAnimator;
    const-string v1, "pathData"

    const/4 v2, 0x1

    invoke-static {p1, p4, v1, v2}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedString(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Ljava/lang/String;

    move-result-object v1

    .line 401
    .local v1, "pathData":Ljava/lang/String;
    if-eqz v1, :cond_4c

    .line 402
    const-string v2, "propertyXName"

    const/4 v3, 0x2

    invoke-static {p1, p4, v2, v3}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedString(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Ljava/lang/String;

    move-result-object v2

    .line 404
    .local v2, "propertyXName":Ljava/lang/String;
    const-string v4, "propertyYName"

    const/4 v5, 0x3

    invoke-static {p1, p4, v4, v5}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedString(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Ljava/lang/String;

    move-result-object v4

    .line 408
    .local v4, "propertyYName":Ljava/lang/String;
    if-eq p2, v3, :cond_1f

    const/4 v3, 0x4

    if-ne p2, v3, :cond_20

    .line 411
    :cond_1f
    const/4 p2, 0x0

    .line 413
    :cond_20
    if-nez v2, :cond_40

    if-eqz v4, :cond_25

    goto :goto_40

    .line 414
    :cond_25
    new-instance v3, Landroid/view/InflateException;

    new-instance v5, Ljava/lang/StringBuilder;

    invoke-direct {v5}, Ljava/lang/StringBuilder;-><init>()V

    invoke-virtual {p1}, Landroid/content/res/TypedArray;->getPositionDescription()Ljava/lang/String;

    move-result-object v6

    invoke-virtual {v5, v6}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v6, " propertyXName or propertyYName is needed for PathData"

    invoke-virtual {v5, v6}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v5}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v5

    invoke-direct {v3, v5}, Landroid/view/InflateException;-><init>(Ljava/lang/String;)V

    throw v3

    .line 417
    :cond_40
    :goto_40
    invoke-static {v1}, Landroid/support/v4/graphics/PathParser;->createPathFromPathData(Ljava/lang/String;)Landroid/graphics/Path;

    move-result-object v3

    .line 418
    .local v3, "path":Landroid/graphics/Path;
    const/high16 v5, 0x3f000000    # 0.5f

    mul-float v5, v5, p3

    invoke-static {v3, v0, v5, v2, v4}, Landroid/support/graphics/drawable/AnimatorInflaterCompat;->setupPathMotion(Landroid/graphics/Path;Landroid/animation/ObjectAnimator;FLjava/lang/String;Ljava/lang/String;)V

    .line 420
    .end local v2    # "propertyXName":Ljava/lang/String;
    .end local v3    # "path":Landroid/graphics/Path;
    .end local v4    # "propertyYName":Ljava/lang/String;
    goto :goto_56

    .line 421
    :cond_4c
    const-string v2, "propertyName"

    const/4 v3, 0x0

    .line 422
    invoke-static {p1, p4, v2, v3}, Landroid/support/v4/content/res/TypedArrayUtils;->getNamedString(Landroid/content/res/TypedArray;Lorg/xmlpull/v1/XmlPullParser;Ljava/lang/String;I)Ljava/lang/String;

    move-result-object v2

    .line 424
    .local v2, "propertyName":Ljava/lang/String;
    invoke-virtual {v0, v2}, Landroid/animation/ObjectAnimator;->setPropertyName(Ljava/lang/String;)V

    .line 428
    .end local v2    # "propertyName":Ljava/lang/String;
    :goto_56
    return-void
.end method

.method private static setupPathMotion(Landroid/graphics/Path;Landroid/animation/ObjectAnimator;FLjava/lang/String;Ljava/lang/String;)V
    .registers 24
    .param p0, "path"    # Landroid/graphics/Path;
    .param p1, "oa"    # Landroid/animation/ObjectAnimator;
    .param p2, "precision"    # F
    .param p3, "propertyXName"    # Ljava/lang/String;
    .param p4, "propertyYName"    # Ljava/lang/String;

    .line 435
    move-object/from16 v0, p0

    move-object/from16 v1, p1

    move-object/from16 v2, p3

    move-object/from16 v3, p4

    new-instance v4, Landroid/graphics/PathMeasure;

    const/4 v5, 0x0

    invoke-direct {v4, v0, v5}, Landroid/graphics/PathMeasure;-><init>(Landroid/graphics/Path;Z)V

    .line 436
    .local v4, "measureForTotalLength":Landroid/graphics/PathMeasure;
    const/4 v6, 0x0

    .line 439
    .local v6, "totalLength":F
    new-instance v7, Ljava/util/ArrayList;

    invoke-direct {v7}, Ljava/util/ArrayList;-><init>()V

    .line 440
    .local v7, "contourLengths":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Ljava/lang/Float;>;"
    const/4 v8, 0x0

    invoke-static {v8}, Ljava/lang/Float;->valueOf(F)Ljava/lang/Float;

    move-result-object v8

    invoke-virtual {v7, v8}, Ljava/util/ArrayList;->add(Ljava/lang/Object;)Z

    .line 442
    :goto_1c
    invoke-virtual {v4}, Landroid/graphics/PathMeasure;->getLength()F

    move-result v8

    .line 443
    .local v8, "pathLength":F
    add-float/2addr v6, v8

    .line 444
    invoke-static {v6}, Ljava/lang/Float;->valueOf(F)Ljava/lang/Float;

    move-result-object v9

    invoke-virtual {v7, v9}, Ljava/util/ArrayList;->add(Ljava/lang/Object;)Z

    .line 446
    .end local v8    # "pathLength":F
    invoke-virtual {v4}, Landroid/graphics/PathMeasure;->nextContour()Z

    move-result v8

    if-nez v8, :cond_cf

    .line 449
    new-instance v8, Landroid/graphics/PathMeasure;

    invoke-direct {v8, v0, v5}, Landroid/graphics/PathMeasure;-><init>(Landroid/graphics/Path;Z)V

    .line 451
    .local v8, "pathMeasure":Landroid/graphics/PathMeasure;
    const/16 v9, 0x64

    div-float v10, v6, p2

    float-to-int v10, v10

    const/4 v11, 0x1

    add-int/2addr v10, v11

    invoke-static {v9, v10}, Ljava/lang/Math;->min(II)I

    move-result v9

    .line 453
    .local v9, "numPoints":I
    new-array v10, v9, [F

    .line 454
    .local v10, "mX":[F
    new-array v12, v9, [F

    .line 455
    .local v12, "mY":[F
    const/4 v13, 0x2

    new-array v14, v13, [F

    .line 457
    .local v14, "position":[F
    const/4 v15, 0x0

    .line 458
    .local v15, "contourIndex":I
    add-int/lit8 v13, v9, -0x1

    int-to-float v13, v13

    div-float v13, v6, v13

    .line 459
    .local v13, "step":F
    const/16 v16, 0x0

    .line 464
    .local v16, "currentDistance":F
    move/from16 v11, v16

    move/from16 v16, v15

    const/4 v15, 0x0

    .local v11, "currentDistance":F
    .local v15, "i":I
    .local v16, "contourIndex":I
    :goto_52
    move/from16 v17, v15

    move/from16 v5, v17

    .end local v15    # "i":I
    .local v5, "i":I
    if-ge v5, v9, :cond_9b

    .line 465
    const/4 v0, 0x0

    invoke-virtual {v8, v11, v14, v0}, Landroid/graphics/PathMeasure;->getPosTan(F[F[F)Z

    .line 467
    const/4 v0, 0x0

    aget v15, v14, v0

    aput v15, v10, v5

    .line 468
    const/4 v0, 0x1

    aget v15, v14, v0

    aput v15, v12, v5

    .line 469
    add-float/2addr v11, v13

    .line 470
    add-int/lit8 v0, v16, 0x1

    move-object/from16 v18, v4

    .end local v4    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    .local v18, "measureForTotalLength":Landroid/graphics/PathMeasure;
    invoke-virtual {v7}, Ljava/util/ArrayList;->size()I

    move-result v4

    if-ge v0, v4, :cond_93

    add-int/lit8 v0, v16, 0x1

    .line 471
    invoke-virtual {v7, v0}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/Float;

    invoke-virtual {v0}, Ljava/lang/Float;->floatValue()F

    move-result v0

    cmpl-float v0, v11, v0

    if-lez v0, :cond_93

    .line 472
    add-int/lit8 v0, v16, 0x1

    invoke-virtual {v7, v0}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/Float;

    invoke-virtual {v0}, Ljava/lang/Float;->floatValue()F

    move-result v0

    sub-float/2addr v11, v0

    .line 473
    add-int/lit8 v16, v16, 0x1

    .line 474
    invoke-virtual {v8}, Landroid/graphics/PathMeasure;->nextContour()Z

    .line 464
    :cond_93
    add-int/lit8 v15, v5, 0x1

    move-object/from16 v4, v18

    move-object/from16 v0, p0

    const/4 v5, 0x0

    goto :goto_52

    .line 479
    .end local v5    # "i":I
    .end local v18    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    .restart local v4    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    :cond_9b
    move-object/from16 v18, v4

    .end local v4    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    .restart local v18    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    const/4 v0, 0x0

    .line 480
    .local v0, "x":Landroid/animation/PropertyValuesHolder;
    const/4 v4, 0x0

    .line 481
    .local v4, "y":Landroid/animation/PropertyValuesHolder;
    if-eqz v2, :cond_a5

    .line 482
    invoke-static {v2, v10}, Landroid/animation/PropertyValuesHolder;->ofFloat(Ljava/lang/String;[F)Landroid/animation/PropertyValuesHolder;

    move-result-object v0

    .line 484
    :cond_a5
    if-eqz v3, :cond_ab

    .line 485
    invoke-static {v3, v12}, Landroid/animation/PropertyValuesHolder;->ofFloat(Ljava/lang/String;[F)Landroid/animation/PropertyValuesHolder;

    move-result-object v4

    .line 487
    :cond_ab
    if-nez v0, :cond_b7

    .line 488
    const/4 v5, 0x1

    new-array v5, v5, [Landroid/animation/PropertyValuesHolder;

    const/4 v15, 0x0

    aput-object v4, v5, v15

    invoke-virtual {v1, v5}, Landroid/animation/ObjectAnimator;->setValues([Landroid/animation/PropertyValuesHolder;)V

    goto :goto_ce

    .line 489
    :cond_b7
    const/4 v5, 0x1

    const/4 v15, 0x0

    if-nez v4, :cond_c3

    .line 490
    new-array v5, v5, [Landroid/animation/PropertyValuesHolder;

    aput-object v0, v5, v15

    invoke-virtual {v1, v5}, Landroid/animation/ObjectAnimator;->setValues([Landroid/animation/PropertyValuesHolder;)V

    goto :goto_ce

    .line 492
    :cond_c3
    const/4 v5, 0x2

    new-array v5, v5, [Landroid/animation/PropertyValuesHolder;

    aput-object v0, v5, v15

    const/4 v15, 0x1

    aput-object v4, v5, v15

    invoke-virtual {v1, v5}, Landroid/animation/ObjectAnimator;->setValues([Landroid/animation/PropertyValuesHolder;)V

    .line 494
    :goto_ce
    return-void

    .line 442
    .end local v0    # "x":Landroid/animation/PropertyValuesHolder;
    .end local v8    # "pathMeasure":Landroid/graphics/PathMeasure;
    .end local v9    # "numPoints":I
    .end local v10    # "mX":[F
    .end local v11    # "currentDistance":F
    .end local v12    # "mY":[F
    .end local v13    # "step":F
    .end local v14    # "position":[F
    .end local v16    # "contourIndex":I
    .end local v18    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    .local v4, "measureForTotalLength":Landroid/graphics/PathMeasure;
    :cond_cf
    move-object/from16 v0, p0

    .end local v4    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    .restart local v18    # "measureForTotalLength":Landroid/graphics/PathMeasure;
    goto/16 :goto_1c
.end method
