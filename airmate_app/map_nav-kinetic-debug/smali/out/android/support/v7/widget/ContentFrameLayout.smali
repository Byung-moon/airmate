.class public Landroid/support/v7/widget/ContentFrameLayout;
.super Landroid/widget/FrameLayout;
.source "ContentFrameLayout.java"


# annotations
.annotation build Landroid/support/annotation/RestrictTo;
    value = {
        .enum Landroid/support/annotation/RestrictTo$Scope;->LIBRARY:Landroid/support/annotation/RestrictTo$Scope;
    }
.end annotation

.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;
    }
.end annotation


# instance fields
.field private mAttachListener:Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

.field private final mDecorPadding:Landroid/graphics/Rect;

.field private mFixedHeightMajor:Landroid/util/TypedValue;

.field private mFixedHeightMinor:Landroid/util/TypedValue;

.field private mFixedWidthMajor:Landroid/util/TypedValue;

.field private mFixedWidthMinor:Landroid/util/TypedValue;

.field private mMinWidthMajor:Landroid/util/TypedValue;

.field private mMinWidthMinor:Landroid/util/TypedValue;


# direct methods
.method public constructor <init>(Landroid/content/Context;)V
    .registers 3
    .param p1, "context"    # Landroid/content/Context;

    .line 57
    const/4 v0, 0x0

    invoke-direct {p0, p1, v0}, Landroid/support/v7/widget/ContentFrameLayout;-><init>(Landroid/content/Context;Landroid/util/AttributeSet;)V

    .line 58
    return-void
.end method

.method public constructor <init>(Landroid/content/Context;Landroid/util/AttributeSet;)V
    .registers 4
    .param p1, "context"    # Landroid/content/Context;
    .param p2, "attrs"    # Landroid/util/AttributeSet;

    .line 61
    const/4 v0, 0x0

    invoke-direct {p0, p1, p2, v0}, Landroid/support/v7/widget/ContentFrameLayout;-><init>(Landroid/content/Context;Landroid/util/AttributeSet;I)V

    .line 62
    return-void
.end method

.method public constructor <init>(Landroid/content/Context;Landroid/util/AttributeSet;I)V
    .registers 5
    .param p1, "context"    # Landroid/content/Context;
    .param p2, "attrs"    # Landroid/util/AttributeSet;
    .param p3, "defStyleAttr"    # I

    .line 65
    invoke-direct {p0, p1, p2, p3}, Landroid/widget/FrameLayout;-><init>(Landroid/content/Context;Landroid/util/AttributeSet;I)V

    .line 66
    new-instance v0, Landroid/graphics/Rect;

    invoke-direct {v0}, Landroid/graphics/Rect;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    .line 67
    return-void
.end method


# virtual methods
.method public dispatchFitSystemWindows(Landroid/graphics/Rect;)V
    .registers 2
    .param p1, "insets"    # Landroid/graphics/Rect;
    .annotation build Landroid/support/annotation/RestrictTo;
        value = {
            .enum Landroid/support/annotation/RestrictTo$Scope;->LIBRARY_GROUP:Landroid/support/annotation/RestrictTo$Scope;
        }
    .end annotation

    .line 74
    invoke-virtual {p0, p1}, Landroid/support/v7/widget/ContentFrameLayout;->fitSystemWindows(Landroid/graphics/Rect;)Z

    .line 75
    return-void
.end method

.method public getFixedHeightMajor()Landroid/util/TypedValue;
    .registers 2

    .line 193
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMajor:Landroid/util/TypedValue;

    if-nez v0, :cond_b

    new-instance v0, Landroid/util/TypedValue;

    invoke-direct {v0}, Landroid/util/TypedValue;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMajor:Landroid/util/TypedValue;

    .line 194
    :cond_b
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMajor:Landroid/util/TypedValue;

    return-object v0
.end method

.method public getFixedHeightMinor()Landroid/util/TypedValue;
    .registers 2

    .line 198
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMinor:Landroid/util/TypedValue;

    if-nez v0, :cond_b

    new-instance v0, Landroid/util/TypedValue;

    invoke-direct {v0}, Landroid/util/TypedValue;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMinor:Landroid/util/TypedValue;

    .line 199
    :cond_b
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMinor:Landroid/util/TypedValue;

    return-object v0
.end method

.method public getFixedWidthMajor()Landroid/util/TypedValue;
    .registers 2

    .line 183
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMajor:Landroid/util/TypedValue;

    if-nez v0, :cond_b

    new-instance v0, Landroid/util/TypedValue;

    invoke-direct {v0}, Landroid/util/TypedValue;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMajor:Landroid/util/TypedValue;

    .line 184
    :cond_b
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMajor:Landroid/util/TypedValue;

    return-object v0
.end method

.method public getFixedWidthMinor()Landroid/util/TypedValue;
    .registers 2

    .line 188
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMinor:Landroid/util/TypedValue;

    if-nez v0, :cond_b

    new-instance v0, Landroid/util/TypedValue;

    invoke-direct {v0}, Landroid/util/TypedValue;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMinor:Landroid/util/TypedValue;

    .line 189
    :cond_b
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMinor:Landroid/util/TypedValue;

    return-object v0
.end method

.method public getMinWidthMajor()Landroid/util/TypedValue;
    .registers 2

    .line 173
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMajor:Landroid/util/TypedValue;

    if-nez v0, :cond_b

    new-instance v0, Landroid/util/TypedValue;

    invoke-direct {v0}, Landroid/util/TypedValue;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMajor:Landroid/util/TypedValue;

    .line 174
    :cond_b
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMajor:Landroid/util/TypedValue;

    return-object v0
.end method

.method public getMinWidthMinor()Landroid/util/TypedValue;
    .registers 2

    .line 178
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMinor:Landroid/util/TypedValue;

    if-nez v0, :cond_b

    new-instance v0, Landroid/util/TypedValue;

    invoke-direct {v0}, Landroid/util/TypedValue;-><init>()V

    iput-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMinor:Landroid/util/TypedValue;

    .line 179
    :cond_b
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMinor:Landroid/util/TypedValue;

    return-object v0
.end method

.method protected onAttachedToWindow()V
    .registers 2

    .line 204
    invoke-super {p0}, Landroid/widget/FrameLayout;->onAttachedToWindow()V

    .line 205
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mAttachListener:Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

    if-eqz v0, :cond_c

    .line 206
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mAttachListener:Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

    invoke-interface {v0}, Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;->onAttachedFromWindow()V

    .line 208
    :cond_c
    return-void
.end method

.method protected onDetachedFromWindow()V
    .registers 2

    .line 212
    invoke-super {p0}, Landroid/widget/FrameLayout;->onDetachedFromWindow()V

    .line 213
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mAttachListener:Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

    if-eqz v0, :cond_c

    .line 214
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mAttachListener:Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

    invoke-interface {v0}, Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;->onDetachedFromWindow()V

    .line 216
    :cond_c
    return-void
.end method

.method protected onMeasure(II)V
    .registers 16
    .param p1, "widthMeasureSpec"    # I
    .param p2, "heightMeasureSpec"    # I

    .line 97
    invoke-virtual {p0}, Landroid/support/v7/widget/ContentFrameLayout;->getContext()Landroid/content/Context;

    move-result-object v0

    invoke-virtual {v0}, Landroid/content/Context;->getResources()Landroid/content/res/Resources;

    move-result-object v0

    invoke-virtual {v0}, Landroid/content/res/Resources;->getDisplayMetrics()Landroid/util/DisplayMetrics;

    move-result-object v0

    .line 98
    .local v0, "metrics":Landroid/util/DisplayMetrics;
    iget v1, v0, Landroid/util/DisplayMetrics;->widthPixels:I

    iget v2, v0, Landroid/util/DisplayMetrics;->heightPixels:I

    if-ge v1, v2, :cond_14

    const/4 v1, 0x1

    goto :goto_15

    :cond_14
    const/4 v1, 0x0

    .line 100
    .local v1, "isPortrait":Z
    :goto_15
    invoke-static {p1}, Landroid/view/View$MeasureSpec;->getMode(I)I

    move-result v2

    .line 101
    .local v2, "widthMode":I
    invoke-static {p2}, Landroid/view/View$MeasureSpec;->getMode(I)I

    move-result v3

    .line 103
    .local v3, "heightMode":I
    const/4 v4, 0x0

    .line 104
    .local v4, "fixedWidth":Z
    const/4 v5, 0x6

    const/4 v6, 0x5

    const/high16 v7, -0x80000000

    const/high16 v8, 0x40000000    # 2.0f

    if-ne v2, v7, :cond_67

    .line 105
    if-eqz v1, :cond_2b

    iget-object v9, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMinor:Landroid/util/TypedValue;

    goto :goto_2d

    :cond_2b
    iget-object v9, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedWidthMajor:Landroid/util/TypedValue;

    .line 106
    .local v9, "tvw":Landroid/util/TypedValue;
    :goto_2d
    if-eqz v9, :cond_67

    iget v10, v9, Landroid/util/TypedValue;->type:I

    if-eqz v10, :cond_67

    .line 107
    const/4 v10, 0x0

    .line 108
    .local v10, "w":I
    iget v11, v9, Landroid/util/TypedValue;->type:I

    if-ne v11, v6, :cond_3e

    .line 109
    invoke-virtual {v9, v0}, Landroid/util/TypedValue;->getDimension(Landroid/util/DisplayMetrics;)F

    move-result v11

    float-to-int v10, v11

    goto :goto_4d

    .line 110
    :cond_3e
    iget v11, v9, Landroid/util/TypedValue;->type:I

    if-ne v11, v5, :cond_4d

    .line 111
    iget v11, v0, Landroid/util/DisplayMetrics;->widthPixels:I

    int-to-float v11, v11

    iget v12, v0, Landroid/util/DisplayMetrics;->widthPixels:I

    int-to-float v12, v12

    invoke-virtual {v9, v11, v12}, Landroid/util/TypedValue;->getFraction(FF)F

    move-result v11

    float-to-int v10, v11

    .line 113
    :cond_4d
    :goto_4d
    if-lez v10, :cond_67

    .line 114
    iget-object v11, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    iget v11, v11, Landroid/graphics/Rect;->left:I

    iget-object v12, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    iget v12, v12, Landroid/graphics/Rect;->right:I

    add-int/2addr v11, v12

    sub-int/2addr v10, v11

    .line 115
    invoke-static {p1}, Landroid/view/View$MeasureSpec;->getSize(I)I

    move-result v11

    .line 116
    .local v11, "widthSize":I
    nop

    .line 117
    invoke-static {v10, v11}, Ljava/lang/Math;->min(II)I

    move-result v12

    .line 116
    invoke-static {v12, v8}, Landroid/view/View$MeasureSpec;->makeMeasureSpec(II)I

    move-result p1

    .line 118
    const/4 v4, 0x1

    .line 123
    .end local v9    # "tvw":Landroid/util/TypedValue;
    .end local v10    # "w":I
    .end local v11    # "widthSize":I
    :cond_67
    if-ne v3, v7, :cond_a9

    .line 124
    if-eqz v1, :cond_6e

    iget-object v9, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMajor:Landroid/util/TypedValue;

    goto :goto_70

    :cond_6e
    iget-object v9, p0, Landroid/support/v7/widget/ContentFrameLayout;->mFixedHeightMinor:Landroid/util/TypedValue;

    .line 125
    .local v9, "tvh":Landroid/util/TypedValue;
    :goto_70
    if-eqz v9, :cond_a9

    iget v10, v9, Landroid/util/TypedValue;->type:I

    if-eqz v10, :cond_a9

    .line 126
    const/4 v10, 0x0

    .line 127
    .local v10, "h":I
    iget v11, v9, Landroid/util/TypedValue;->type:I

    if-ne v11, v6, :cond_81

    .line 128
    invoke-virtual {v9, v0}, Landroid/util/TypedValue;->getDimension(Landroid/util/DisplayMetrics;)F

    move-result v11

    float-to-int v10, v11

    goto :goto_90

    .line 129
    :cond_81
    iget v11, v9, Landroid/util/TypedValue;->type:I

    if-ne v11, v5, :cond_90

    .line 130
    iget v11, v0, Landroid/util/DisplayMetrics;->heightPixels:I

    int-to-float v11, v11

    iget v12, v0, Landroid/util/DisplayMetrics;->heightPixels:I

    int-to-float v12, v12

    invoke-virtual {v9, v11, v12}, Landroid/util/TypedValue;->getFraction(FF)F

    move-result v11

    float-to-int v10, v11

    .line 132
    :cond_90
    :goto_90
    if-lez v10, :cond_a9

    .line 133
    iget-object v11, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    iget v11, v11, Landroid/graphics/Rect;->top:I

    iget-object v12, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    iget v12, v12, Landroid/graphics/Rect;->bottom:I

    add-int/2addr v11, v12

    sub-int/2addr v10, v11

    .line 134
    invoke-static {p2}, Landroid/view/View$MeasureSpec;->getSize(I)I

    move-result v11

    .line 135
    .local v11, "heightSize":I
    nop

    .line 136
    invoke-static {v10, v11}, Ljava/lang/Math;->min(II)I

    move-result v12

    .line 135
    invoke-static {v12, v8}, Landroid/view/View$MeasureSpec;->makeMeasureSpec(II)I

    move-result p2

    .line 141
    .end local v9    # "tvh":Landroid/util/TypedValue;
    .end local v10    # "h":I
    .end local v11    # "heightSize":I
    :cond_a9
    invoke-super {p0, p1, p2}, Landroid/widget/FrameLayout;->onMeasure(II)V

    .line 143
    invoke-virtual {p0}, Landroid/support/v7/widget/ContentFrameLayout;->getMeasuredWidth()I

    move-result v9

    .line 144
    .local v9, "width":I
    const/4 v10, 0x0

    .line 146
    .local v10, "measure":Z
    invoke-static {v9, v8}, Landroid/view/View$MeasureSpec;->makeMeasureSpec(II)I

    move-result p1

    .line 148
    if-nez v4, :cond_f3

    if-ne v2, v7, :cond_f3

    .line 149
    if-eqz v1, :cond_be

    iget-object v7, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMinor:Landroid/util/TypedValue;

    goto :goto_c0

    :cond_be
    iget-object v7, p0, Landroid/support/v7/widget/ContentFrameLayout;->mMinWidthMajor:Landroid/util/TypedValue;

    .line 150
    .local v7, "tv":Landroid/util/TypedValue;
    :goto_c0
    if-eqz v7, :cond_f3

    iget v11, v7, Landroid/util/TypedValue;->type:I

    if-eqz v11, :cond_f3

    .line 151
    const/4 v11, 0x0

    .line 152
    .local v11, "min":I
    iget v12, v7, Landroid/util/TypedValue;->type:I

    if-ne v12, v6, :cond_d1

    .line 153
    invoke-virtual {v7, v0}, Landroid/util/TypedValue;->getDimension(Landroid/util/DisplayMetrics;)F

    move-result v5

    float-to-int v11, v5

    goto :goto_e0

    .line 154
    :cond_d1
    iget v6, v7, Landroid/util/TypedValue;->type:I

    if-ne v6, v5, :cond_e0

    .line 155
    iget v5, v0, Landroid/util/DisplayMetrics;->widthPixels:I

    int-to-float v5, v5

    iget v6, v0, Landroid/util/DisplayMetrics;->widthPixels:I

    int-to-float v6, v6

    invoke-virtual {v7, v5, v6}, Landroid/util/TypedValue;->getFraction(FF)F

    move-result v5

    float-to-int v11, v5

    .line 157
    :cond_e0
    :goto_e0
    if-lez v11, :cond_ec

    .line 158
    iget-object v5, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    iget v5, v5, Landroid/graphics/Rect;->left:I

    iget-object v6, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    iget v6, v6, Landroid/graphics/Rect;->right:I

    add-int/2addr v5, v6

    sub-int/2addr v11, v5

    .line 160
    :cond_ec
    if-ge v9, v11, :cond_f3

    .line 161
    invoke-static {v11, v8}, Landroid/view/View$MeasureSpec;->makeMeasureSpec(II)I

    move-result p1

    .line 162
    const/4 v10, 0x1

    .line 167
    .end local v7    # "tv":Landroid/util/TypedValue;
    .end local v11    # "min":I
    :cond_f3
    if-eqz v10, :cond_f8

    .line 168
    invoke-super {p0, p1, p2}, Landroid/widget/FrameLayout;->onMeasure(II)V

    .line 170
    :cond_f8
    return-void
.end method

.method public setAttachListener(Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;)V
    .registers 2
    .param p1, "attachListener"    # Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

    .line 78
    iput-object p1, p0, Landroid/support/v7/widget/ContentFrameLayout;->mAttachListener:Landroid/support/v7/widget/ContentFrameLayout$OnAttachListener;

    .line 79
    return-void
.end method

.method public setDecorPadding(IIII)V
    .registers 6
    .param p1, "left"    # I
    .param p2, "top"    # I
    .param p3, "right"    # I
    .param p4, "bottom"    # I
    .annotation build Landroid/support/annotation/RestrictTo;
        value = {
            .enum Landroid/support/annotation/RestrictTo$Scope;->LIBRARY_GROUP:Landroid/support/annotation/RestrictTo$Scope;
        }
    .end annotation

    .line 89
    iget-object v0, p0, Landroid/support/v7/widget/ContentFrameLayout;->mDecorPadding:Landroid/graphics/Rect;

    invoke-virtual {v0, p1, p2, p3, p4}, Landroid/graphics/Rect;->set(IIII)V

    .line 90
    invoke-static {p0}, Landroid/support/v4/view/ViewCompat;->isLaidOut(Landroid/view/View;)Z

    move-result v0

    if-eqz v0, :cond_e

    .line 91
    invoke-virtual {p0}, Landroid/support/v7/widget/ContentFrameLayout;->requestLayout()V

    .line 93
    :cond_e
    return-void
.end method
