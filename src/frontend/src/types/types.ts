

export type Coordinate = {
    x: number;
    y: number;
};

export type Stroke = Coordinate[];

export type UserInput = {
    strokes: Stroke[],
    shape_id: number|null,
    shape_type: string,
    shapetype_code: number|null,
    params_to_vary: number|null
    param_values: number|null
}
