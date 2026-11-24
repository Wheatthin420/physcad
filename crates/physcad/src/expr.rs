//! Hand-written arithmetic expression parser for model parameters.
//!
//! Grammar (recursive descent, classic precedence):
//!
//! ```text
//!   expr    := term (('+' | '-') term)*
//!   term    := unary (('*' | '/' | '%') unary)*
//!   unary   := ('-' | '+') unary | power
//!   power   := atom ('^' unary)?          // right-assoc; allows 2^-1
//!   atom    := NUMBER | IDENT | IDENT '(' [expr (',' expr)*] ')' | '(' expr ')'
//! ```
//!
//! All evaluation is f64. `-2^2 == -4` (unary minus binds looser than `^`,
//! the math convention). Errors carry 1-based line/col from the source.

use std::collections::BTreeMap;
use std::fmt;

/// Parse or evaluation failure with a source position.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExprError {
    pub line: usize,
    pub col: usize,
    pub msg: String,
}

impl ExprError {
    fn at(line: usize, col: usize, msg: impl Into<String>) -> Self {
        Self {
            line,
            col,
            msg: msg.into(),
        }
    }
}

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}: {}", self.line, self.col, self.msg)
    }
}

impl std::error::Error for ExprError {}

#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    Const(f64),
    Var {
        name: String,
        line: usize,
        col: usize,
    },
    Unary {
        op: char,
        operand: Box<Expr>,
    },
    Binary {
        op: char,
        lhs: Box<Expr>,
        rhs: Box<Expr>,
    },
    Call {
        func: String,
        args: Vec<Expr>,
        line: usize,
        col: usize,
    },
}

// ---------------------------------------------------------------- lexer ----

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Num(f64),
    Ident(String),
    Sym(char),
}

#[derive(Clone, Debug)]
struct Spanned {
    tok: Token,
    line: usize,
    col: usize,
}

fn tokenize(src: &str) -> Result<Vec<Spanned>, ExprError> {
    let bytes = src.as_bytes();
    let mut i = 0usize;
    let mut line = 1usize;
    let mut col = 1usize;
    let mut out = Vec::new();

    // (byte offset, line, col) tracking
    let bump = |i: &mut usize, line: &mut usize, col: &mut usize| {
        if bytes[*i] == b'\n' {
            *line += 1;
            *col = 1;
        } else {
            *col += 1;
        }
        *i += 1;
    };

    while i < bytes.len() {
        let c = bytes[i];
        let start_line = line;
        let start_col = col;
        match c {
            b' ' | b'\t' | b'\r' | b'\n' => bump(&mut i, &mut line, &mut col),
            b'0'..=b'9' | b'.' => {
                let mut num = String::new();
                let mut dots = 0;
                while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == b'.') {
                    if bytes[i] == b'.' {
                        dots += 1;
                    }
                    if dots > 1 {
                        return Err(ExprError::at(
                            start_line,
                            start_col,
                            "number has more than one '.'",
                        ));
                    }
                    num.push(bytes[i] as char);
                    bump(&mut i, &mut line, &mut col);
                }
                // Optional exponent: 1e3, 1e+3, 0.5e-2 — only if it parses.
                if i < bytes.len() && (bytes[i] | 0x20) == b'e' {
                    let mut j = i + 1;
                    if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
                        j += 1;
                    }
                    if j < bytes.len() && bytes[j].is_ascii_digit() {
                        num.push(bytes[i] as char);
                        bump(&mut i, &mut line, &mut col);
                        num.push(bytes[i] as char);
                        bump(&mut i, &mut line, &mut col);
                        while i < bytes.len() && bytes[i].is_ascii_digit() {
                            num.push(bytes[i] as char);
                            bump(&mut i, &mut line, &mut col);
                        }
                    }
                }
                let v: f64 = num.parse().map_err(|_| {
                    ExprError::at(start_line, start_col, format!("bad number {num:?}"))
                })?;
                out.push(Spanned {
                    tok: Token::Num(v),
                    line: start_line,
                    col: start_col,
                });
            }
            b'A'..=b'Z' | b'a'..=b'z' | b'_' => {
                let mut name = String::new();
                while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
                    name.push(bytes[i] as char);
                    bump(&mut i, &mut line, &mut col);
                }
                out.push(Spanned {
                    tok: Token::Ident(name),
                    line: start_line,
                    col: start_col,
                });
            }
            b'+' | b'-' | b'*' | b'/' | b'^' | b'%' | b'(' | b')' | b',' => {
                out.push(Spanned {
                    tok: Token::Sym(c as char),
                    line: start_line,
                    col: start_col,
                });
                bump(&mut i, &mut line, &mut col);
            }
            other => {
                return Err(ExprError::at(
                    start_line,
                    start_col,
                    format!("unexpected character {:?}", other as char),
                ));
            }
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------- parser ---

struct Parser {
    toks: Vec<Spanned>,
    pos: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.toks.get(self.pos).map(|s| &s.tok)
    }

    fn peek_span(&self) -> Option<(usize, usize)> {
        self.toks.get(self.pos).map(|s| (s.line, s.col))
    }

    fn eat_sym(&mut self, c: char) -> bool {
        if self.peek() == Some(&Token::Sym(c)) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn expect_sym(&mut self, c: char) -> Result<(), ExprError> {
        if self.eat_sym(c) {
            Ok(())
        } else {
            let (line, col) = self
                .peek_span()
                .unwrap_or_else(|| self.toks.last().map_or((1, 1), |s| (s.line, s.col + 1)));
            Err(ExprError::at(line, col, format!("expected '{c}'")))
        }
    }

    fn parse_expr(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.parse_term()?;
        while let Some(Token::Sym('+')) | Some(Token::Sym('-')) = self.peek() {
            let op = match self.peek() {
                Some(Token::Sym(c)) => *c,
                _ => unreachable!(),
            };
            self.pos += 1;
            let rhs = self.parse_term()?;
            lhs = Expr::Binary {
                op,
                lhs: Box::new(lhs),
                rhs: Box::new(rhs),
            };
        }
        Ok(lhs)
    }

    fn parse_term(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.parse_unary()?;
        while let Some(Token::Sym('*')) | Some(Token::Sym('/')) | Some(Token::Sym('%')) =
            self.peek()
        {
            let op = match self.peek() {
                Some(Token::Sym(c)) => *c,
                _ => unreachable!(),
            };
            self.pos += 1;
            let rhs = self.parse_unary()?;
            lhs = Expr::Binary {
                op,
                lhs: Box::new(lhs),
                rhs: Box::new(rhs),
            };
        }
        Ok(lhs)
    }

    fn parse_unary(&mut self) -> Result<Expr, ExprError> {
        if self.eat_sym('-') {
            let operand = self.parse_unary()?;
            return Ok(Expr::Unary {
                op: '-',
                operand: Box::new(operand),
            });
        }
        if self.eat_sym('+') {
            return self.parse_unary();
        }
        self.parse_power()
    }

    fn parse_power(&mut self) -> Result<Expr, ExprError> {
        let base = self.parse_atom()?;
        if self.eat_sym('^') {
            // right-associative: exponent may itself start with unary minus
            let exp = self.parse_unary()?;
            return Ok(Expr::Binary {
                op: '^',
                lhs: Box::new(base),
                rhs: Box::new(exp),
            });
        }
        Ok(base)
    }

    fn parse_atom(&mut self) -> Result<Expr, ExprError> {
        match self.peek().cloned() {
            Some(Token::Num(v)) => {
                self.pos += 1;
                Ok(Expr::Const(v))
            }
            Some(Token::Ident(name)) => {
                let (line, col) = self.peek_span().unwrap();
                self.pos += 1;
                if self.peek() == Some(&Token::Sym('(')) {
                    self.pos += 1;
                    let mut args = Vec::new();
                    if self.eat_sym(')') {
                        return Err(ExprError::at(
                            line,
                            col,
                            format!("{name}() takes arguments"),
                        ));
                    }
                    loop {
                        args.push(self.parse_expr()?);
                        if self.eat_sym(',') {
                            continue;
                        }
                        break;
                    }
                    self.expect_sym(')')?;
                    Ok(Expr::Call {
                        func: name,
                        args,
                        line,
                        col,
                    })
                } else {
                    Ok(Expr::Var { name, line, col })
                }
            }
            Some(Token::Sym('(')) => {
                self.pos += 1;
                let e = self.parse_expr()?;
                self.expect_sym(')')?;
                Ok(e)
            }
            Some(Token::Sym(c)) => {
                let (line, col) = self.peek_span().unwrap();
                Err(ExprError::at(line, col, format!("unexpected '{c}'")))
            }
            None => {
                let (line, col) = self.toks.last().map_or((1, 1), |s| (s.line, s.col + 1));
                Err(ExprError::at(line, col, "expression ends early"))
            }
        }
    }
}

/// Parse one full expression; trailing garbage is an error.
pub fn parse(src: &str) -> Result<Expr, ExprError> {
    let toks = tokenize(src)?;
    if toks.is_empty() {
        return Err(ExprError::at(1, 1, "empty expression"));
    }
    let mut p = Parser { toks, pos: 0 };
    let e = p.parse_expr()?;
    if let Some((line, col)) = p.peek_span() {
        return Err(ExprError::at(
            line,
            col,
            "unexpected input after expression",
        ));
    }
    Ok(e)
}

// ------------------------------------------------------------- evaluation --

impl Expr {
    /// Evaluate against a variable scope. Built-in constants (`pi`, `tau`,
    /// `e`) fill in when a scope entry does not exist. Any non-finite
    /// result or domain violation is an error carrying this node's span.
    pub fn eval(&self, scope: &BTreeMap<String, f64>) -> Result<f64, ExprError> {
        match self {
            Expr::Const(v) => Ok(*v),
            Expr::Var { name, line, col } => {
                if let Some(v) = scope.get(name) {
                    Ok(*v)
                } else {
                    match name.as_str() {
                        "pi" => Ok(std::f64::consts::PI),
                        "tau" => Ok(std::f64::consts::TAU),
                        "e" => Ok(std::f64::consts::E),
                        _ => Err(ExprError::at(
                            *line,
                            *col,
                            format!("unknown variable {name:?}"),
                        )),
                    }
                }
            }
            Expr::Unary { op, operand } => {
                let v = operand.eval(scope)?;
                Ok(match op {
                    '-' => -v,
                    '+' => v,
                    _ => unreachable!("unary op {op} cannot reach eval"),
                })
            }
            Expr::Binary { op, lhs, rhs } => {
                let a = lhs.eval(scope)?;
                let b = rhs.eval(scope)?;
                let v = match op {
                    '+' => a + b,
                    '-' => a - b,
                    '*' => a * b,
                    '/' => {
                        if b == 0.0 {
                            return Err(ExprError::at(
                                expr_span(rhs),
                                expr_col(rhs),
                                "division by zero",
                            ));
                        }
                        a / b
                    }
                    '%' => {
                        if b == 0.0 {
                            return Err(ExprError::at(
                                expr_span(rhs),
                                expr_col(rhs),
                                "modulo by zero",
                            ));
                        }
                        a % b
                    }
                    '^' => a.powf(b),
                    _ => unreachable!("binary op {op} cannot reach eval"),
                };
                Self::finite_check(self, v, "result")
            }
            Expr::Call { func, args, .. } => {
                let vals: Result<Vec<f64>, ExprError> =
                    args.iter().map(|a| a.eval(scope)).collect();
                let vals = vals?;
                let arity_ok = match func.as_str() {
                    "min" | "max" | "pow" | "atan2" | "hypot" | "mod" => vals.len() == 2,
                    _ => vals.len() == 1,
                };
                if !arity_ok {
                    return Err(ExprError::at(
                        *self.line_of(),
                        *self.col_of(),
                        format!("{func}() expects a different number of arguments"),
                    ));
                }
                let (a, b) = (vals[0], vals.get(1).copied());
                let known = match func.as_str() {
                    "abs" => Some(a.abs()),
                    "sqrt" => {
                        if a < 0.0 {
                            None
                        } else {
                            Some(a.sqrt())
                        }
                    }
                    "cbrt" => Some(a.cbrt()),
                    "ln" => {
                        if a <= 0.0 {
                            None
                        } else {
                            Some(a.ln())
                        }
                    }
                    "log10" => {
                        if a <= 0.0 {
                            None
                        } else {
                            Some(a.log10())
                        }
                    }
                    "exp" => Some(a.exp()),
                    "floor" => Some(a.floor()),
                    "ceil" => Some(a.ceil()),
                    "sin" => Some(a.sin()),
                    "cos" => Some(a.cos()),
                    "tan" => Some(a.tan()),
                    "asin" => {
                        if a < -1.0 || a > 1.0 {
                            None
                        } else {
                            Some(a.asin())
                        }
                    }
                    "acos" => {
                        if a < -1.0 || a > 1.0 {
                            None
                        } else {
                            Some(a.acos())
                        }
                    }
                    "atan" => Some(a.atan()),
                    "min" => Some(a.min(b.unwrap())),
                    "max" => Some(a.max(b.unwrap())),
                    "pow" => Some(a.powf(b.unwrap())),
                    "atan2" => Some(a.atan2(b.unwrap())),
                    "hypot" => Some(a.hypot(b.unwrap())),
                    "mod" => {
                        if b == Some(0.0) {
                            None
                        } else {
                            Some(a % b.unwrap())
                        }
                    }
                    _ => None,
                };
                match known {
                    Some(v) => Self::finite_check(self, v, func),
                    None => Err(ExprError::at(
                        *self.line_of(),
                        *self.col_of(),
                        format!("{func}() domain error (argument {a:?})"),
                    )),
                }
            }
        }
    }

    fn finite_check(&self, v: f64, what: &str) -> Result<f64, ExprError> {
        if v.is_finite() {
            Ok(v)
        } else {
            Err(ExprError::at(
                *self.line_of(),
                *self.col_of(),
                format!("{what} produced non-finite value"),
            ))
        }
    }

    /// Left-most source span inside this subtree (0,0 only for pure
    /// Const/Operator nodes, which always sit under a spanned parent).
    fn line_of(&self) -> &usize {
        match self {
            Expr::Var { line, .. } | Expr::Call { line, .. } => line,
            Expr::Unary { operand, .. } => operand.line_of(),
            Expr::Binary { lhs, .. } => lhs.line_of(),
            Expr::Const(_) => &0,
        }
    }

    fn col_of(&self) -> &usize {
        match self {
            Expr::Var { col, .. } | Expr::Call { col, .. } => col,
            Expr::Unary { operand, .. } => operand.col_of(),
            Expr::Binary { lhs, .. } => lhs.col_of(),
            Expr::Const(_) => &0,
        }
    }
}

fn expr_span(e: &Expr) -> usize {
    *e.line_of()
}

fn expr_col(e: &Expr) -> usize {
    *e.col_of()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn scope(pairs: &[(&str, f64)]) -> BTreeMap<String, f64> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    fn ev(src: &str, scope: &BTreeMap<String, f64>) -> f64 {
        parse(src)
            .unwrap_or_else(|e| panic!("{src}: {e}"))
            .eval(scope)
            .unwrap_or_else(|e| panic!("{src}: {e}"))
    }

    fn err(src: &str) -> ExprError {
        parse(src)
            .and_then(|e| e.eval(&BTreeMap::new()))
            .expect_err("{src} must fail")
    }

    #[test]
    fn precedence_and_associativity() {
        let s = BTreeMap::new();
        assert_eq!(ev("2+3*4", &s), 14.0);
        assert_eq!(ev("(2+3)*4", &s), 20.0);
        assert_eq!(ev("2^3^2", &s), 512.0, "^ must be right-associative");
        assert_eq!(ev("-2^2", &s), -4.0, "unary minus binds looser than ^");
        assert_eq!(ev("2^-1", &s), 0.5, "^ may take a unary-minus exponent");
        assert_eq!(ev("10%4", &s), 2.0);
        assert_eq!(ev("10/4/2", &s), 1.25, "/ is left-associative");
        assert_eq!(ev("7/2%2", &s), 1.5, "% binds like *");
        assert_eq!(ev("--3", &s), 3.0);
        assert_eq!(ev("+-2", &s), -2.0);
        assert_eq!(ev("3-4+5", &s), 4.0, "+/- left-associative");
    }

    #[test]
    fn numbers_and_float_forms() {
        let s = BTreeMap::new();
        assert_eq!(ev("1.5", &s), 1.5);
        assert_eq!(ev(".25", &s), 0.25);
        assert_eq!(ev("1e3", &s), 1000.0);
        assert_eq!(ev("2.5e-2", &s), 0.025);
        assert_eq!(ev("1E+2", &s), 100.0);
    }

    #[test]
    fn variables_and_constants() {
        let s = scope(&[("w", 12.0), ("h", 3.0)]);
        assert_eq!(ev("w/h", &s), 4.0);
        assert_eq!(ev("w*h + 1", &s), 37.0);
        assert!((ev("pi", &s) - PI).abs() < 1e-15);
        assert!((ev("tau", &s) - 2.0 * PI).abs() < 1e-15);
        assert!((ev("e", &s) - std::f64::consts::E).abs() < 1e-15);
        // a user parameter shadows nothing weirdly; user value wins
        let s2 = scope(&[("pi", 3.0)]);
        assert_eq!(ev("pi*2", &s2), 6.0);
    }

    #[test]
    fn functions() {
        let s = BTreeMap::new();
        assert!((ev("sqrt(2)*sqrt(2) - 2", &s)).abs() < 1e-12);
        assert_eq!(ev("min(3,7) + max(3,7)", &s), 10.0);
        assert_eq!(ev("pow(2,10)", &s), 1024.0);
        assert_eq!(ev("hypot(3,4)", &s), 5.0);
        assert_eq!(ev("abs(-4) + floor(-1.2) + ceil(1.2)", &s), 4.0 - 2.0 + 2.0);
        assert!((ev("sin(pi/2)", &s) - 1.0).abs() < 1e-15);
        assert!(
            (ev("atan2(-1,-1) + 3.0*pi/4.0", &s)).abs() < 1e-12,
            "atan3 quadrant"
        );
        assert_eq!(ev("mod(-7,4)", &s), -7.0 % 4.0);
        assert!((ev("cbrt(27) - 3", &s)).abs() < 1e-12);
        assert!((ev("ln(e)", &s) - 1.0).abs() < 1e-15);
        assert!((ev("log10(1000) - 3", &s)).abs() < 1e-15);
        // nested calls + expressions as arguments
        assert!((ev("sqrt(min(4, pow(3,2))) * 2", &s) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn parse_errors_carry_positions() {
        let e = err("1+*2");
        assert_eq!((e.line, e.col), (1, 3), "{e:?}");
        assert!(e.msg.contains("unexpected '*'"));

        let e = err("(1+2");
        assert!(e.msg.contains("expected ')'"), "{e}");

        let e = err("2 3");
        assert_eq!((e.line, e.col), (1, 3), "{e:?}");
        assert!(e.msg.contains("after expression"));

        let e = err("");
        assert!(e.msg.contains("empty"));

        let e = err("1.2.3");
        assert!(e.msg.contains("more than one '.'"), "{e}");

        // multi-line tokenizing keeps line/col honest
        let e = err("1 +\n2 $ 3");
        assert_eq!((e.line, e.col), (2, 3), "{e:?}");
        assert!(e.msg.contains("unexpected character '$'"), "{e}");
    }

    #[test]
    fn eval_errors_carry_positions() {
        let e = err("sqrt(-1)");
        assert!(e.msg.contains("sqrt() domain error"), "{e}");
        let e = err("1/0");
        assert!(e.msg.contains("division by zero"), "{e}");
        let e = err("5%0");
        assert!(e.msg.contains("modulo by zero"), "{e}");
        let e = err("asin(2)");
        assert!(e.msg.contains("asin() domain"), "{e}");
        let e = err("ln(0)");
        assert!(e.msg.contains("ln() domain"), "{e}");
        let e = err("foo + 1");
        assert!(e.msg.contains("unknown variable \"foo\""), "{e}");
        let e = err("min(1,2,3)");
        assert!(e.msg.contains("min() expects"), "{e}");
        let e = err("sin()");
        assert!(e.msg.contains("takes arguments"), "{e}");
        // a lone inf literal parses (Rust float parsing saturates); the
        // non-finite guard only surfaces once arithmetic consumes it:
        assert_eq!(
            parse("1e400").and_then(|p| p.eval(&BTreeMap::new())),
            Ok(f64::INFINITY)
        );
        let e = err("1e308*1e308");
        assert!(e.msg.contains("non-finite"), "{e}");
    }

    #[test]
    fn ast_shape_is_deterministic_data() {
        // The tree itself is typed data (mission lesson: ordering/precedence
        // bugs become test-visible). Pin a non-trivial shape.
        let e = parse("a*b + sin(c)/2 - 1").unwrap();
        let want = Expr::Binary {
            op: '-',
            lhs: Box::new(Expr::Binary {
                op: '+',
                lhs: Box::new(Expr::Binary {
                    op: '*',
                    lhs: Box::new(Expr::Var {
                        name: "a".into(),
                        line: 1,
                        col: 1,
                    }),
                    rhs: Box::new(Expr::Var {
                        name: "b".into(),
                        line: 1,
                        col: 3,
                    }),
                }),
                rhs: Box::new(Expr::Binary {
                    op: '/',
                    lhs: Box::new(Expr::Call {
                        func: "sin".into(),
                        args: vec![Expr::Var {
                            name: "c".into(),
                            line: 1,
                            col: 11,
                        }],
                        line: 1,
                        col: 7,
                    }),
                    rhs: Box::new(Expr::Const(2.0)),
                }),
            }),
            rhs: Box::new(Expr::Const(1.0)),
        };
        assert_eq!(e, want);
    }
}
