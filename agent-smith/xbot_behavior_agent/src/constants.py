def enum(**enums):
    return type('Enum', (), enums)

Outcomes = enum(Success='success', Failed='failed', Aborted='aborted')
ALL_OUTCOMES = [ Outcomes.Success, Outcomes.Failed, Outcomes.Aborted ]
STATUS_CODES = {
    Outcomes.Success: 0,
    Outcomes.Failed: 1,
    Outcomes.Aborted: 2
}